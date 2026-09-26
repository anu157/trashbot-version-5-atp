# trashbot-version-5-atp
# trashbot-claires-version
# gthack13-
# TrashBot 🗑️🤖

An autonomous, person-following trash collection robot. TrashBot trails behind you, accepts trash you hand to it, sorts it, and delivers it to the nearest trash bin — all without you lifting a finger (or leaving the couch).

![status](https://img.shields.io/badge/status-in%20development-yellow)
![platform](https://img.shields.io/badge/platform-Raspberry%20Pi%204B-c51a4a)
![license](https://img.shields.io/badge/license-MIT-blue)

---

## Overview

TrashBot is a small, cube-shaped robot (noticeably smaller than a standard trash can) that follows its user around a space, collects trash items handed to it, temporarily stores them internally, sorts them by type, and autonomously navigates to the appropriate trash/recycling bin to deposit the load — using a spring-loaded lift-and-tilt mechanism to raise the trash up to bin height and empty itself in.

**Core behaviors:**
- 👤 **Follows the user** around the room/environment
- 🖐️ **Accepts trash** handed to it, storing it internally in the cube body
- 🧠 **Sorts trash** (e.g., recyclable vs. general waste)
- 🗺️ **Maps and navigates** the environment, locating trash/recycling bins
- ⬆️ **Lifts and dumps** trash via a spring-extending arm that raises up to the bin's rim and tilts to empty
- 📍 **Self-localizes** — always knows where it is relative to the user and the bins

---

## How It Works

1. **Follow mode** — TrashBot uses a camera + person-detection model to identify and track its user, maintaining a safe following distance while avoiding obstacles.
2. **Trash intake** — When the user presents trash near the robot (proximity/gesture or touch trigger), a hatch/opening on the cube accepts the item into an internal holding compartment.
3. **Sorting** — Onboard sensors (camera + weight/material sensing) classify the item (e.g., recyclable, compost, general waste) and route it to the correct internal bin/compartment.
4. **Bin-finding & navigation** — Once full (or on a schedule/command), TrashBot switches to navigation mode, using its pre-built map (or SLAM) plus known bin locations to path-find to the nearest matching trash can.
5. **Dumping** — At the bin, a spring-loaded extending mechanism telescopes upward until it reaches/clears the rim of the (much larger) trash can, then tilts the internal compartment to empty the contents inside.
6. **Return** — TrashBot re-localizes and returns to following the user.

---

## Hardware

| Component | Purpose |
|---|---|
| Raspberry Pi 4B | Main compute — vision, navigation, control logic |
| Camera module (Pi Cam / USB cam) | Person detection & tracking, trash classification, visual odometry |
| Ultrasonic / IR / ToF sensors | Obstacle avoidance, bin-rim detection, proximity trash intake |
| Motor driver + drive motors/wheels | Locomotion |
| IMU (optional) | Orientation stability, dead-reckoning assist |
| Spring-loaded linear actuator / extending mechanism | Lifts internal trash compartment up to bin height |
| Servo (tilt mechanism) | Tilts compartment to empty trash into bin |
| Weight / load sensor (optional) | Detects trash intake and fullness |
| Battery pack (LiPo or similar) + power management | Portable power |
| Chassis | Cube-shaped body, sized smaller than a standard trash can |

> 📌 Fill in exact part numbers/models as your build is finalized (motor torque, actuator stroke length, camera FOV, etc.)

> No LIDAR and no wheel encoders are used for localization — position is estimated with **monocular visual odometry** off the same camera used for person tracking. See [Visual Odometry](#visual-odometry) below.

---

## Software Stack

- **OS:** Raspberry Pi OS (64-bit recommended for Pi 4B)
- **Language:** Python (primary), with optional C/C++ for performance-critical control loops
- **Computer Vision:** OpenCV + a lightweight object/person detection model (e.g., MobileNet-SSD, YOLO-tiny, or similar edge-friendly model)
- **Visual Odometry:** Custom sparse optical-flow + essential-matrix pipeline (OpenCV), scale-corrected via the existing AprilTag — see [Visual Odometry](#visual-odometry)
- **Navigation / Mapping:** SLAM (e.g., via ROS2 + a lidar/depth sensor) or a simpler waypoint-based map if lidar isn't used
- **Robot Framework (optional):** ROS2 for sensor fusion, navigation stack, and modular control
- **Control:** GPIO / PWM control for motors, servo, and linear actuator via Python (`RPi.GPIO`, `pigpio`, or `gpiozero`)

---

## Repository Structure

```
trashbot/
├── vision/              # Person detection & trash classification models
├── navigation/          # Mapping, localization, path planning
├── control/             # Motor, actuator, and servo control scripts
├── sorting/             # Trash classification logic
├── docs/                # Wiring diagrams, CAD files, build notes
├── tests/               # Unit / hardware-in-loop tests
├── main.py              # Entry point — state machine (Follow / Intake / Sort / Navigate / Dump)
└── README.md
```

---

## Current Implementation (HackGT build)

| File | What it does |
|---|---|
| `main.py` | Main loop: camera → detection → state machine → motors, plus debug window |
| `state_machine.py` | FOLLOW / FIND_PERSON / GO_TO_BIN / SEARCH_BIN / APPROACH_BIN / THROW |
| `vision.py` | Camera (USB or phone stream), person AprilTag tracking, YOLO-World trash-can detection |
| `navigation.py` | Dead-reckoning pose + remembered trash-can locations (`bins.json`) |
| `visual_odometry.py` | Monocular visual odometry — sparse optical flow + essential matrix, scale-corrected via the AprilTag's known size. Feeds real position deltas into `navigation.py` in place of (or blended with) commanded-speed dead reckoning. |
| `calibrate_camera.py` | One-time checkerboard camera calibration; produces the `camera_matrix`/`dist_coeffs` `visual_odometry.py` needs |
| `test_visual_odometry.py` | Standalone webcam test harness for `visual_odometry.py` — no robot required |
| `robot.py` | Serial link to the Arduino; falls back to simulation if none is connected |
| `config.py` | Every tunable number (camera, speeds, distances, calibration) |
| `arduino/trashbot/trashbot.ino` | TB6612 motors, ultrasonic, dump servo, trash sensor, 500 ms safety stop |

**Behavior:** the user wears AprilTag `tag36h11` ID 0 (print it ~15 cm wide). The robot follows it, and whenever the camera sees a trash can it records the can's position. Once the compartment sensor detects trash for 1.5 s, it drives back to the nearest recorded can, lines up using the camera and ultrasonic, dumps, and goes back to the person.

**Arduino wiring:**

| Arduino pin | Connects to |
|---|---|
| 5 / 7 / 8 | TB6612 PWMA / AIN1 / AIN2 (left motor) |
| 6 / 9 / 10 | TB6612 PWMB / BIN1 / BIN2 (right motor) |
| 4 | TB6612 STBY |
| 11 / 12 | HC-SR04 TRIG / ECHO |
| 3 | Dump servo signal |
| A0 | IR sensor pointed into the compartment |

**Laptop test:** `python3 main.py` with no Arduino plugged in runs in simulation. Hold the tag up to the webcam, point it at a trash can, and press `t` to fake "trash loaded".

---

## Visual Odometry

TrashBot has no wheel encoders and no LIDAR, so position isn't tracked by "wheel turns" or a scanned map — it's tracked by watching the camera feed move. This is **monocular visual odometry (VO)**: a lightweight, hand-rolled OpenCV pipeline (not ROS, not viso2/OpenVINS/ORB-SLAM — see rationale below), living in `visual_odometry.py`.

**How it works:**
1. Track sparse corner features frame-to-frame with Lucas-Kanade optical flow.
2. Recover rotation + a *unit-scale* translation direction from the essential matrix (5-point algorithm).
3. Resolve real metric scale using the AprilTag already worn by the user: since the tag's physical size is known, `solvePnP` gives a true camera-to-tag distance every frame the tag is visible, and the change in that distance rescales the unit translation into meters.
4. When the tag briefly isn't visible, the last known scale is held over (constant-velocity assumption) until the tag reappears.

**Why not viso2_ros / OpenVINS / ORB-SLAM2/3?** Those are the "proven," optimized-C++, typically ROS-wrapped options, and are the natural upgrade path if drift or frame rate becomes a real problem. For a hackathon build, a plain Python/OpenCV script was faster to get running and easier to debug end-to-end, at the cost of being slower per-frame and drifting more over long runs than a full SLAM stack would.

### 1. Calibrate the camera (one time, required)

VO needs an accurate camera matrix or the recovered rotation/translation will be subtly wrong in ways that are hard to diagnose later. Do this once per camera:

1. Print a 9×6-internal-corner checkerboard, tape it flat to something rigid.
2. Run:
   ```bash
   python3 calibrate_camera.py
   ```
3. Move the checkerboard in front of the camera — near/far, tilted in every direction, covering all four corners of the frame, not just the center. Press **SPACE** to capture each pose (aim for 15–20 captures).
4. Press **c** to run calibration. It prints a reprojection error (under ~0.5 is good, redo if over ~1.0) and saves `camera_matrix` / `dist_coeffs` to `camera_calibration.npz`.
5. Load those two arrays wherever `VisualOdometry(...)` is constructed (see `test_visual_odometry.py` for the exact usage).

### 2. Test it stand-alone (no robot needed)

```bash
python3 test_visual_odometry.py
```

- Print the AprilTag at a **known, ruler-measured** size and set `TAG_SIZE_M` in the script to match (printers don't always print at exactly 100% scale — measure it, don't assume).
- Lay out a tape measure, move the camera exactly 1.0 m along it while keeping the tag in view, and compare the printed accumulated distance to the real 1.0 m.
  - Within ~5–10% → good enough to drive bin-return navigation.
  - Way off (2×, 10×) → check `TAG_SIZE_M` and the camera calibration first; those are the two most common causes.
- Watch the on-screen FPS: above ~15 FPS on the Pi 4B, the plain-Python pipeline is fine as-is. Below ~5 FPS, cut `max_corners` in `VisualOdometry.__init__`, lower the camera resolution, or consider moving to viso2_ros/ORB-SLAM3's compiled C++ pipeline instead.

### 3. Integrate into navigation

`visual_odometry.py`'s `update()` returns a per-frame `(dx, dy, dtheta, scale_valid)` delta. Feed that into `navigation.py`'s pose accumulator **in place of** (or blended with) the current "commanded speed × elapsed time" dead reckoning, which is currently the dominant source of position error:

```python
dx, dy, dtheta, scale_valid = vo.update(frame, gray, tag_corners, TAG_SIZE_M)
pose.x     += dx * cos(pose.theta) - dy * sin(pose.theta)
pose.y     += dx * sin(pose.theta) + dy * cos(pose.theta)
pose.theta += dtheta
```

If `scale_valid` is `False` for many consecutive frames (tag out of view for a while, e.g. during `GO_TO_BIN`), consider cross-checking against the ultrasonic sensor's readings, or widening your position uncertainty if you later add an EKF.

---

## Getting Started

### Prerequisites
- Raspberry Pi 4B (4GB+ recommended) flashed with Raspberry Pi OS
- Python 3.9+
- Assembled chassis with actuator, motors, and sensors wired per `docs/wiring.md`

### Installation
```bash
git clone https://github.com/<your-username>/trashbot.git
cd trashbot
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### Running
```bash
python3 main.py
```

On startup, TrashBot will calibrate its sensors, attempt to locate the user, and enter **Follow mode**.

---

## Roadmap

- [ ] Person-following with obstacle avoidance
- [ ] Trash intake detection (gesture/touch-triggered hatch)
- [ ] Trash sorting classifier (recyclable vs. general waste)
- [ ] Bin location mapping & path planning
- [ ] Spring-extend + tilt dumping mechanism
- [ ] Full autonomous loop (follow → collect → sort → navigate → dump → return)
- [ ] Battery level & fullness monitoring with low-battery/return-to-dock behavior
- [ ] Mobile app / voice command interface
- [ ] Camera calibration + visual odometry integrated into `navigation.py`'s live pose estimate
- [ ] Evaluate viso2_ros / ORB-SLAM3 if VO drift or frame rate becomes a bottleneck

---

## Contributing

Contributions, issues, and feature requests are welcome! Feel free to check the [issues page](../../issues) or open a pull request.

---

## License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.

---

## Acknowledgments

Built with a Raspberry Pi 4B, a lot of hot glue, and an unreasonable dislike of taking out the trash.
