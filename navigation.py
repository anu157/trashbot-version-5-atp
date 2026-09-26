"""Where the robot is (dead reckoning) and where the trash cans are."""
import json
import math
import os
import time
from dataclasses import asdict, dataclass

import config


@dataclass
class Pose:
    x: float = 0.0      # meters, +x = the direction the robot faced at startup
    y: float = 0.0      # meters, +y = to the left of that
    theta: float = 0.0  # radians, counter-clockwise


class Odometry:
    """Estimates pose from motor commands. No encoders, so it drifts: use it
    to get roughly back to a trash can, then let the camera take over."""

    def __init__(self):
        self.pose = Pose()

    def update(self, left_pwm, right_pwm, dt):
        v = (left_pwm + right_pwm) / 2 * config.M_PER_S_PER_PWM
        w = (right_pwm - left_pwm) / 2 * config.RAD_PER_S_PER_PWM
        p = self.pose
        p.x += v * math.cos(p.theta) * dt
        p.y += v * math.sin(p.theta) * dt
        p.theta = wrap_angle(p.theta + w * dt)

    def snapshot(self):
        return Pose(self.pose.x, self.pose.y, self.pose.theta)


class VisualOdometryTracker:
    """Estimates pose from the camera (visualodometry_simple.VisualOdometry)
    instead of commanded PWM. Scale comes from the AprilTag on the person's
    back whenever it's visible (see VisualOdometry docstring); between
    sightings the last known scale is held over, so this alone still drifts
    -- just less, and in a different way, than pure PWM dead reckoning.

    Falls back to one tick of PWM integration only when VO returns nothing
    at all for a frame (e.g. cold start, or optical flow lost every feature
    -- a fully dark frame, a hard camera jolt). This keeps `pose` moving
    sensibly through the rare tick VO can't handle, rather than freezing.
    """

    def __init__(self, camera_matrix, dist_coeffs=None):
        from visualodometry_simple import VisualOdometry
        self.vo = VisualOdometry(camera_matrix, dist_coeffs)
        self.pose = Pose()
        self.frames_without_scale_fix = 0  # ticks since the tag was last visible

    def update(self, frame, gray, tag_corners, tag_size_m, left_pwm, right_pwm, dt):
        """Call once per camera frame. tag_corners: obs.person.corners, or
        None if the person's tag isn't visible this frame. Returns
        scale_valid (True if this tick's scale came from a fresh tag
        sighting, False if held-over or PWM-fallback) so callers can decide
        how much to trust the pose, e.g. widen a search radius after many
        False ticks in a row."""
        dx, dy, dtheta, scale_valid = self.vo.update(frame, gray, tag_corners, tag_size_m)

        if dx == 0.0 and dy == 0.0 and dtheta == 0.0:
            # VO produced nothing usable this tick -- fall back to PWM
            # integration for just this one tick rather than standing still.
            v = (left_pwm + right_pwm) / 2 * config.M_PER_S_PER_PWM
            w = (right_pwm - left_pwm) / 2 * config.RAD_PER_S_PER_PWM
            dy = v * dt      # PWM model's "forward" maps onto VO's forward axis
            dx = 0.0
            dtheta = w * dt

        p = self.pose
        p.x += dx * math.cos(p.theta) - dy * math.sin(p.theta)
        p.y += dx * math.sin(p.theta) + dy * math.cos(p.theta)
        p.theta = wrap_angle(p.theta + dtheta)

        self.frames_without_scale_fix = 0 if scale_valid else self.frames_without_scale_fix + 1
        return scale_valid

    def snapshot(self):
        return Pose(self.pose.x, self.pose.y, self.pose.theta)


def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def to_world(pose, bearing, distance):
    """Convert a camera sighting (relative to the robot) to map coordinates."""
    angle = pose.theta + bearing
    return pose.x + distance * math.cos(angle), pose.y + distance * math.sin(angle)


@dataclass
class TrashCan:
    id: int
    x: float
    y: float
    sightings: int = 1
    last_seen: float = 0.0


class BinMap:
    """Remembered trash can locations, saved to disk between runs."""

    def __init__(self, path=config.BIN_MAP_FILE):
        self.path = path
        self.bins = []
        if path and os.path.exists(path):
            with open(path) as f:
                self.bins = [TrashCan(**b) for b in json.load(f)]
            print(f"[MAP] loaded {len(self.bins)} trash can(s) from {path}")

    def record(self, x, y):
        """Add a sighting. Nearby sightings are averaged into one trash can."""
        now = time.time()
        for b in self.bins:
            if math.hypot(b.x - x, b.y - y) < config.BIN_MERGE_RADIUS_M:
                # running average, capped so a moved can eventually updates
                n = min(b.sightings, 10)
                b.x = (b.x * n + x) / (n + 1)
                b.y = (b.y * n + y) / (n + 1)
                b.sightings += 1
                b.last_seen = now
                self._save()
                return b
        b = TrashCan(id=len(self.bins), x=x, y=y, last_seen=now)
        self.bins.append(b)
        print(f"[MAP] new trash can #{b.id} at ({x:.2f}, {y:.2f})")
        self._save()
        return b

    def nearest(self, pose):
        if not self.bins:
            return None
        return min(self.bins, key=lambda b: math.hypot(b.x - pose.x, b.y - pose.y))

    def _save(self):
        if self.path:
            with open(self.path, "w") as f:
                json.dump([asdict(b) for b in self.bins], f, indent=2)
