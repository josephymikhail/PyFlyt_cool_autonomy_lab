
#spawn a drone in the center. flies up then completes movement in a square

import matplotlib.pyplot as plt
import numpy as np
from PyFlyt.core import Aviary, loadOBJ, obj_collision, obj_visual
import pybullet as p
import time
import math


# initialize the log
#this sets the limit for the length of the simulation
drone_log = np.zeros((1000, 3), dtype=np.float32)
load_log = np.zeros((1000, 3), dtype=np.float32)
angle_log = np.zeros((1000, 3), dtype=np.float32)

 # the starting position and orientations
start_pos = np.array([[0.0, 0.0, 1.5]])
start_orn = np.array([[0.0, 0.0, 0.0]])

# environment setup
env = Aviary(start_pos=start_pos, start_orn=start_orn, render=True, drone_type="quadx")

#spawns drone with box constrained to it
box_path = "/Users/josephmikhail/Desktop/utaustin/coolautonomy/PyFlyt_cool_autonomy_lab/PyFlyt/models/blue_tea_box/box.urdf"
def initialize_sim(box_path):




    #ensure you are editing local copy
    # print("param_path:", env.drones[0].param_path)
    # print("drone_path:", env.drones[0].drone_path)


    #confirms that we are already using crazyflie urdf
    #print("Drone path:", env.drones[0].drone_path)

    # set to position control
    env.set_mode(6)

    #from https://github.com/c-yiting/pybullet-URDF-models/tree/main/urdf_models/models/blue_tea_box
    #box_path = "/Users/josephmikhail/Desktop/utaustin/coolautonomy/PyFlyt_cool_autonomy_lab/PyFlyt/models/blue_tea_box/box.urdf"

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

#computes string tilt (between drone and load)
def compute_string_angle(drone_pos, load_pos):
    drone_pos = np.array(drone_pos)
    load_pos = np.array(load_pos)
    vector = drone_pos - load_pos
    x,y,z = vector[0], vector[1], vector[2]
    string_angle = math.atan2(math.sqrt(x*x + y*y), z) #output in radians
    string_angle = string_angle * 57.2958
    return string_angle
    #r = drone_pos - load_pos
    #breakpoint()

#makes system move in square-ish trajectory
def move_in_square(velocities):
    #to do: write function that goes in square motion
    previous_rope = p.addUserDebugLine([0,0,0], [0,0,0], [1,0,0], 3, lifeTime = 0, physicsClientId=env._client)
    index = 0
    for i in range(1000):
        env.set_setpoint(0, velocities[index])
        env.step()
        if (i % 250 == 0 and i > 1):
            index += 1

        #track position of drone and load
        drone_pos = p.getBasePositionAndOrientation(1)[0]
        load_pos = p.getBasePositionAndOrientation(2)[0]
        drone_log[i] = drone_pos[0]
        load_log[i] = load_pos[0]
        if i % 25 == 0:
            print("String angle:", compute_string_angle(drone_pos, load_pos))
        angle_log[i] = compute_string_angle(drone_pos, load_pos)

        #draw rope
        p.removeUserDebugItem(previous_rope)
        current_rope = p.addUserDebugLine(drone_pos, load_pos, [1,0,0], 5, lifeTime=0, physicsClientId=env._client)
        previous_rope = current_rope


initialize_sim(box_path)

#i am using mode 6 - [vx, vy, vr, vz]
vel1 = np.array([0.5, 0.0, 0.0, 0.0])
vel2 = np.array([0.0, 0.5, 0.0, 0.0])
vel3 = np.array([-0.5, 0.0, 0.0, 0.0])
vel4 = np.array([0.0, -0.5, 0.0, 0.0])
velocities = [vel1, vel2, vel3, vel4]
pause = np.array([0.0, 0.0, 0.0, 0.0])

move_in_square(velocities)

# # plot stuff out
#blue is x, orange is y, green is z
plt.plot(np.arange(1000), angle_log)
plt.show()

input("Press Enter to close...")