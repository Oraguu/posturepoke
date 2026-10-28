import json
import math
import threading
import time

import cv2
import mediapipe as mp
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

# arm
PORT = "COM4"
ROBOT_ID = "posture_arm"   # same id as calibration
TORQUE_LIMIT = 800         # /1000, lower than this and it sags
MAX_STEP = 10.0            # max deg per command, has to be a float
RATE = 40.0                # commands/sec

# posture
THRESHOLD = 6              # deg off baseline = bad
HOLD_TIME = 15             # sec of bad posture before poke
COOLDOWN = 30              # sec between pokes
MIN_VIS = 0.5              # min landmark confidence
SMOOTH = 0.1               # ema weight, lower = smoother

# camera is on my left so use left side landmarks
mp_pose = mp.solutions.pose
EYE = mp_pose.PoseLandmark.LEFT_EYE
SHOULDER = mp_pose.PoseLandmark.LEFT_SHOULDER

# HOME / POKE from record_poses.py
with open("poses.json") as f:
    poses = json.load(f)

robot = SO101Follower(SO101FollowerConfig(port=PORT, id=ROBOT_ID, max_relative_target=MAX_STEP))
robot.connect()

# torque limit resets on power off so set it every time
for motor in robot.bus.motors:
    robot.bus.write("Torque_Limit", motor, TORQUE_LIMIT)

abort = threading.Event()    # x key
poking = threading.Event()   # arm is busy


class Aborted(Exception):
    pass


def current_pose():
    obs = robot.get_observation()
    return {k: float(v) for k, v in obs.items() if k.endswith(".pos")}


def move_to(target, seconds, settle=True, abortable=True):
    # lerp from current pose to target
    start = current_pose()
    steps = max(1, int(seconds * RATE))
    for i in range(1, steps + 1):
        if abortable and abort.is_set():
            raise Aborted()
        t = i / steps
        robot.send_action({k: start[k] + (target[k] - start[k]) * t for k in target})
        time.sleep(1 / RATE)

    # wait to actually get there (within 4 deg, ignore gripper, 1s max)
    if settle:
        deadline = time.time() + 1
        while time.time() < deadline:
            if abortable and abort.is_set():
                raise Aborted()
            now = current_pose()
            if all(abs(now[k] - target[k]) < 4 for k in target if k != "gripper.pos"):
                break
            robot.send_action(target)
            time.sleep(1 / RATE)


def poke_sequence():
    # separate thread so the camera doesn't freeze
    try:
        move_to(poses["POKE"], 3, settle=False)  # don't wait on contact
        time.sleep(0.1)
    except Aborted:
        print("Poke aborted, retreating")
    finally:
        # always retreat, can't abort this part
        move_to(poses["HOME"], 2, abortable=False)
        abort.clear()
        poking.clear()


def angle_from_vertical(top, bottom, w, h):
    # convert to pixels first or aspect ratio messes up the angle
    dx = (top.x - bottom.x) * w
    dy = (bottom.y - top.y) * h  # y is flipped in images
    return math.degrees(math.atan2(dx, dy))


move_to(poses["HOME"], 3, abortable=False)

cap = cv2.VideoCapture(0)
baseline = None
smooth = None
bad_since = None
last_poke = 0

try:
    with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            h, w = frame.shape[:2]

            # mediapipe needs rgb
            results = pose.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            now = time.time()

            visible = False
            if results.pose_landmarks:
                lm = results.pose_landmarks.landmark
                eye, sh = lm[EYE], lm[SHOULDER]
                if eye.visibility > MIN_VIS and sh.visibility > MIN_VIS:
                    visible = True
                    angle = angle_from_vertical(eye, sh, w, h)
                    smooth = angle if smooth is None else (1 - SMOOTH) * smooth + SMOOTH * angle
                    p_eye = (int(eye.x * w), int(eye.y * h))
                    p_sh = (int(sh.x * w), int(sh.y * h))
                    cv2.line(frame, p_eye, p_sh, (0, 255, 0), 3)

            if not visible:
                text = "can't see eye/shoulder"
                bad_since = None
            elif baseline is None:
                text = "press b while sitting straight"
            else:
                dev = abs(smooth - baseline)
                text = f"dev: {dev:.1f}"
                if dev > THRESHOLD:
                    # start timer, poke if it's been bad long enough
                    if bad_since is None:
                        bad_since = now
                    text += f" | bad {now - bad_since:.0f}s"
                    ready = now - last_poke > COOLDOWN and not poking.is_set()
                    if now - bad_since > HOLD_TIME and ready:
                        poking.set()
                        threading.Thread(target=poke_sequence, daemon=True).start()
                        last_poke = now
                        bad_since = None
                else:
                    bad_since = None

            if poking.is_set():
                text += " | POKING"
            cooldown_left = COOLDOWN - (now - last_poke)
            if last_poke and cooldown_left > 0:
                text += f" | cooldown {cooldown_left:.0f}s"

            cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow("Posture Poke", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('b') and visible:
                baseline = smooth
            elif key == ord('x'):
                abort.set()
            elif key == ord('q'):
                break

finally:
    # finish retreating, then disconnect arm before anything else
    abort.set()
    while poking.is_set():
        time.sleep(0.05)
    robot.disconnect()
    cap.release()
    cv2.destroyAllWindows()