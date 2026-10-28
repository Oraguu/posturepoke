# Posture Poke

A webcam watches my posture from the side. If I slouch for too long, an SO-101 robot arm reaches over and pokes me.

Pose tracking uses pretrained MediaPipe, and the arm uses just uses recorded joint angles since I sit in the same spot every time.

## How it works

1. **Sense:** MediaPipe Pose finds my left eye and left shoulder in a side-view webcam feed.
2. **Decide:** it measures the angle of the eye→shoulder line from vertical and compares it to a baseline I set while sitting straight. Angles stay the same no matter how close I sit, unlike pixel distances. The reading is smoothed with an EMA, and a poke only fires after bad posture holds for a set time (debounce), with a cooldown between pokes.
3. **Act:** the arm interpolates through three hand-recorded poses: HOME → APPROACH → POKE → back. The poke runs on a background thread so the camera keeps running.

## Hardware

- SO-101 arm (Feetech servos), calibrated with LeRobot
- Webcam

## Setup

```
pip install -r requirements.txt
```

LeRobot installs `opencv-python-headless`, which breaks `cv2.imshow`. If you get a "function is not implemented" error:

```
pip uninstall -y opencv-python-headless opencv-python
pip install opencv-python
```

Calibrate the arm (once):

```
lerobot-find-port
lerobot-calibrate --robot.type=so101_follower --robot.port=COM4 --robot.id=posture_arm
```

## Usage

1. `python record_poses.py`: torque turns off, move the arm by hand, and press h / a / p to save HOME / APPROACH / POKE. Press q to save `poses.json`.
2. `python move_test.py`: test the poke sequence with nobody in the chair.
3. `python posture_poke.py`: sit straight, press **b** to set your baseline. Press **x** to abort a poke and **q** to quit.

All tunable numbers (threshold, hold time, cooldown, torque, speed) are at the top of `posture_poke.py`.

## Safety

This is a motor pointed at a person. The script caps per-step joint movement, sets a servo torque limit, and always retreats after a poke. Still: test without a person first, keep the arm's reach away from your face, and keep the power plug within reach.
