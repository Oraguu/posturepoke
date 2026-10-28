import json
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

robot = SO101Follower(SO101FollowerConfig(port="COM4", id="posture_arm"))
robot.connect()
robot.bus.disable_torque()  # arm goes limp, hold it!

poses = {}
names = {"h": "HOME", "a": "APPROACH", "p": "POKE"}

while True:
    key = input("h / a / p to save, q to quit: ")
    if key == "q":
        break
    if key in names:
        obs = robot.get_observation()
        poses[names[key]] = {k: float(v) for k, v in obs.items() if k.endswith(".pos")}
        print("saved", names[key])

json.dump(poses, open("poses.json", "w"), indent=2)
robot.disconnect()