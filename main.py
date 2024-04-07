# Nano: ssh -X tnp@192.168.1.160

import serial
import time
import numpy as np
from generate_graph_opt import get_path


class motionPrim:
    def __init__(
        self, prim_id, start_angle, endpose, costmult, inter_poses, num_interposes
    ):
        self.id = prim_id
        self.start_angle = start_angle
        self.endpose = endpose
        self.cost = costmult
        self.inter_poses = inter_poses
        self.num_interposes = num_interposes
        self.cells_covered = list()


class motionPrims:
    def __init__(self, prims, num_prims, resolution, num_angles):
        self.prims = prims
        self.num_prims = num_prims
        self.resolution = resolution
        self.num_angles = num_angles


def get_prims(prims_file):
    f = open(prims_file, "r")
    resolution = float(f.readline().split(":")[1])
    num_angles = int(f.readline().split(":")[1])
    num_prims = int(f.readline().split(":")[1])
    motion_prims = list()
    for n in range(num_prims):
        prim_id = int(f.readline().split(":")[1])
        prim_id = n
        start_angle = int(f.readline().split(":")[1])
        endpose = [int(n) for n in f.readline().split(":")[1].split()]
        # multcost = 1 if int(f.readline().split(":")[1]) < 5 else 10
        multcost = int(f.readline().split(":")[1]) * 10 + (
            abs(endpose[0]) + abs(endpose[1])
        )
        num_interposes = int(f.readline().split(":")[1])
        interposes = list()
        for i in range(num_interposes):
            interposes.append([float(n.strip()) for n in f.readline().split()])
        motion_prims.append(
            motionPrim(
                prim_id, start_angle, endpose, multcost, interposes, num_interposes
            )
        )
    f.close()
    return motionPrims(motion_prims, num_prims, resolution, num_angles)


arduino = serial.Serial("COM6", 9600, timeout=1)


prims = get_prims("prims_4angles.txt")
prims_dict = dict()
for prim in prims.prims:
    prims_dict[prim.id] = prim


def plan(
    start=[5, 5, 1],
    goal=[88, 120, 1],
    map_size=[114, 122],
    obstacles=[[52, 96, 0, 10], [96, 114, 0, 16], [0, 63, 114, 122]],
):
    start = [5, 5, 1]
    curr_pos = start
    prim_id_commands = get_path(
        map_size=[114, 122],
        obstacles=[[52, 96, 0, 10], [96, 114, 0, 16], [0, 63, 114, 122]],
        start=start,
        goal=[88, 120, 1],
        prims="prims_4angles.txt",
    )
    prim = prims_dict[prim_id_commands[0]]
    curr_pos = [
        curr_pos[0] + prim.endpose[0],
        curr_pos[1] + prim.endpose[1],
        prim.endpose[2],
    ]

    arduino_fw_conversion = prims.resolution / 0.2
    arduino_t_conversion = 1 / (78 * np.pi / 180)
    cmd_sequence = ""
    prev_cmd = ""
    fw_sum = 0
    for id in prim_id_commands:
        prim = prims_dict[id]
        # fw command if angle doesn't change
        if prim.start_angle == prim.endpose[2]:
            if prim.endpose[0] == 0:
                dist = abs(prim.endpose[1]) * arduino_fw_conversion
            else:
                dist = abs(prim.endpose[0]) * arduino_fw_conversion
            cmd = "fw" + str(dist)
            if prev_cmd != "fw":
                prev_cmd = "fw"

        # t command since robot is turning
        else:
            angle = (
                (prim.endpose[2] - prim.start_angle)
                * (np.pi * 2)
                / prims.num_angles
                * arduino_t_conversion
            )
            cmd = f"t{angle:.3f}"
        if cmd[:2] == "fw" and prev_cmd == "fw":
            fw_sum += dist
        else:
            cmd_sequence += "fw" + str(fw_sum) + ":" + cmd + ":"
            fw_sum = 0
            prev_cmd = "t"
    if prev_cmd == "fw":
        cmd_sequence += "fw" + str(fw_sum) + ":"
    print(cmd_sequence)
    print(cmd_sequence.split(":"))
    return cmd_sequence.split(":")[0]


# arduino.write(bytes(cmd_sequence + "\r\n", "utf-8"))
time.sleep(3)
while True:
    cmd = plan()
    print(cmd)
    arduino.write(bytes(cmd + ":\r\n", "utf-8"))
    while True:
        line = arduino.readline()
        print(line)
        if b"complete" in line:
            arduino.write(bytes("odo:" + "\r\n", "utf-8"))
            while True:
                line = arduino.readline()
                if b"x, y, theta" in line:
                    x = str(arduino.readline())
                    y = str(arduino.readline())
                    z = str(arduino.readline())
                    print(x, y, z)
                    exit()


exit()
while True:
    input_str = input("Enter command ")
    arduino.write(bytes(input_str + "\r\n", "utf-8"))
