"""
visual_odometry.py — Monocular visual odometry for TrashBot.

No LIDAR, no wheel encoders. Uses:
  1. Sparse optical flow (Lucas-Kanade) to track features frame-to-frame.
  2. The essential matrix (5-point algorithm) to recover rotation and a
     UNIT-SCALE translation direction.
  3. The existing AprilTag (tag36h11, ID 0, worn by the user) to recover
     real metric scale whenever it's visible, via solvePnP.
  4. A constant-scale fallback for frames where the tag isn't visible.

Integration:
  Call `vo.update(frame, gray)` once per camera frame from vision.py's main
  loop. It returns a (dx, dy, dtheta) delta in meters/radians, in the
  camera's own frame at the time of the previous call. Feed that into
  navigation.py's pose accumulator INSTEAD OF (or blended with) the current
  "commanded speed * elapsed time" dead-reckoning estimate:

      dx, dy, dtheta, scale_valid = vo.update(frame, gray, tag_corners, tag_size_m)
      pose.x     += dx * cos(pose.theta) - dy * sin(pose.theta)
      pose.y     += dx * sin(pose.theta) + dy * cos(pose.theta)
      pose.theta += dtheta

  If scale_valid is False for many consecutive frames, trust the wheel/PWM
  based estimate more (or widen your uncertainty if you add an EKF later).
"""

import cv2
import numpy as np
import time


class VisualOdometry:
    def __init__(self, camera_matrix, dist_coeffs=None, max_corners=200):
        """
        camera_matrix: 3x3 np.array, from cv2.calibrateCamera (do this once —
                       a printed 9x6 checkerboard + calibrate.py is enough).
        dist_coeffs:   lens distortion coefficients (optional, None = assume
                       already-undistorted / negligible distortion).
        """
        self.K = camera_matrix
        self.dist = dist_coeffs if dist_coeffs is not None else np.zeros(5)

        self.max_corners = max_corners
        self.lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )

        self.prev_gray = None
        self.prev_pts = None
        self.last_tag_distance_m = None   # metric distance to tag, last time seen
        self.last_unit_translation = None  # unit-scale t from previous essential-matrix solve
        self.scale = 1.0                   # running scale estimate (meters per "unit")
        self.frames_since_tag = 0

    def _detect_features(self, gray):
        pts = cv2.goodFeaturesToTrack(
            gray, maxCorners=self.max_corners, qualityLevel=0.01, minDistance=8
        )
        return pts

    def _estimate_tag_distance(self, tag_corners, tag_size_m):
        """tag_corners: 4x2 np.array of the AprilTag's image corners (from your
        existing vision.py detector). Returns metric distance in meters via PnP."""
        half = tag_size_m / 2.0
        obj_pts = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float32)
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts, tag_corners.astype(np.float32), self.K, self.dist
        )
        if not ok:
            return None
        return float(np.linalg.norm(tvec))

    def update(self, frame_bgr, gray, tag_corners=None, tag_size_m=0.15):
        """
        Returns (dx, dy, dtheta, scale_valid):
            dx, dy   -- translation in meters, in the PREVIOUS camera frame's
                        x/z ground-plane axes (dx = lateral, dy = forward).
            dtheta   -- rotation about the vertical axis, in radians.
            scale_valid -- True if this update used a fresh metric scale fix
                        (tag was visible this frame), False if it's using the
                        held-over scale from a previous sighting.
        """
        if self.prev_gray is None:
            self.prev_gray = gray
            self.prev_pts = self._detect_features(gray)
            return 0.0, 0.0, 0.0, False

        if self.prev_pts is None or len(self.prev_pts) < 8:
            self.prev_pts = self._detect_features(self.prev_gray)
            if self.prev_pts is None:
                self.prev_gray = gray
                return 0.0, 0.0, 0.0, False

        # 1. Track features into the new frame
        next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_pts, None, **self.lk_params
        )
        good_prev = self.prev_pts[status.flatten() == 1]
        good_next = next_pts[status.flatten() == 1]

        dx = dy = dtheta = 0.0
        scale_valid = False

        if len(good_prev) >= 8:
            # 2. Essential matrix -> unit-scale rotation + translation
            E, mask = cv2.findEssentialMat(
                good_next, good_prev, self.K, method=cv2.RANSAC,
                prob=0.999, threshold=1.0
            )
            if E is not None and E.shape == (3, 3):
                _, R, t, _ = cv2.recoverPose(E, good_next, good_prev, self.K)
                dtheta = float(np.arctan2(R[0, 2], R[2, 2]))  # yaw component

                # 3. Resolve metric scale
                cur_tag_distance = None
                if tag_corners is not None:
                    cur_tag_distance = self._estimate_tag_distance(tag_corners, tag_size_m)

                if cur_tag_distance is not None and self.last_tag_distance_m is not None:
                    delta_tag_distance = abs(self.last_tag_distance_m - cur_tag_distance)
                    if delta_tag_distance > 0.01:  # 1cm — ignore noise-level "moves"
                        self.scale = delta_tag_distance
                        scale_valid = True

                if cur_tag_distance is not None:
                    self.last_tag_distance_m = cur_tag_distance
                    self.frames_since_tag = 0
                else:
                    self.frames_since_tag += 1

                dx = float(t[0]) * self.scale
                dy = float(t[2]) * self.scale  # forward axis in OpenCV camera coords

        # Re-seed features periodically / when flow degrades
        self.prev_gray = gray
        self.prev_pts = good_next.reshape(-1, 1, 2) if len(good_next) >= 8 else self._detect_features(gray)

        return dx, dy, dtheta, scale_valid
