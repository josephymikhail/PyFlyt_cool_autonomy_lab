
#spawn a drone in the center. flies up then completes movement in a square

import matplotlib.pyplot as plt
import numpy as np
from PyFlyt.core import Aviary, loadOBJ, obj_collision, obj_visual
import pybullet as p


# initialize the log
#this sets the limit for the length of the simulation
log = np.zeros((2000, 3), dtype=np.float32)

# the starting position and orientations
start_pos = np.array([[0.0, 0.0, 1.5]])
start_orn = np.array([[0.0, 0.0, 0.0]])

# environment setup
env = Aviary(start_pos=start_pos, start_orn=start_orn, render=True, drone_type="quadx")

#confirms that we are already using crazyflie urdf
#print("Drone path:", env.drones[0].drone_path)

# set to position control
env.set_mode(7)

#from https://github.com/c-yiting/pybullet-URDF-models/tree/main/urdf_models/models/blue_tea_box
box_path = "/home/rk32226/Desktop/joseph/PyFlyt/PyFlyt/models/blue_tea_box/box.urdf"

box = p.loadURDF(
    box_path,
    basePosition=[0.0, 0.0, 1],                          # spawn above ground
    baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),   # yaw/pitch/roll in radians
    useFixedBase=False,
    globalScaling=1,
    flags=p.URDF_USE_INERTIA_FROM_FILE | p.URDF_MERGE_FIXED_LINKS,
    physicsClientId=env._client,
)

env.register_all_new_bodies()

drone_id = 1
#got from "env.print_all_bodies()"" which prints bodyUniqueId and name for everything


# 4) Create a point-to-point (ball) joint = "string"
rope_length = 0.5
drone_pivot = [0.0, 0.0, -rope_length]   # hook point on drone (in drone base frame)
box_pivot   = [0.0, 0.0, 0.05]  # top center of box (in box frame), adjust to your geometry
cid = p.createConstraint(
    parentBodyUniqueId=drone_id, parentLinkIndex=-1,
    childBodyUniqueId=box,   childLinkIndex=-1,
    jointType=p.JOINT_POINT2POINT,
    jointAxis=[0,0,0],
    parentFramePosition=drone_pivot,
    childFramePosition=box_pivot,
    physicsClientId=env._client,
)
p.changeConstraint(cid, maxForce=1e5, physicsClientId=env._client)

#this chunk actually does the movement of the drone
#figure out cleaner way to write this 
pos1 = np.array([0.0, 0.0, 0.0, 1.2])
pos2 = np.array([0.0, 1.0, 0.0, 1.2])
pos3= np.array([1.0, 1.0, 0.0, 1.2])
pos4 = np.array([1.0, 0.0, 0.0, 1.2])

#we are using mode 7 which requires x,y,r(yaw),z as input
env.set_setpoint(0, pos1)
for i in range(500):
    env.step()
    # record the linear position state
    log[i] = env.state(0)[-1]

env.set_setpoint(0, pos2)
for i in range(500, 1000):
    env.step()
    # record the linear position state
    log[i] = env.state(0)[-1]

env.set_setpoint(0, pos3)
for i in range(1000, 1500):
    env.step()
    # record the linear position state
    log[i] = env.state(0)[-1]

env.set_setpoint(0, pos4)
for i in range(1500, 2000):
    env.step()
    # record the linear position state
    log[i] = env.state(0)[-1]

input("Press Enter to close...")

# # plot stuff out
# plt.plot(np.arange(1000), log)
# plt.show()
