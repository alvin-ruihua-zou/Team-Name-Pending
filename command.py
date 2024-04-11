# Nano: ssh -X tnp@192.168.1.160

import serial
import time
import numpy as np
from generate_graph_opt import get_path
import re


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
                prim_id, start_angle, endpose, multcost, interposes, num_interposesi
            )
        )
    f.close()
    return motionPrims(motion_prims, num_prims, resolution, num_angles)


arduino = serial.Serial("COM11", 9600, timeout=1)


# prims = get_prims("prims_4angles.txt")
# prims_dict = dict()
# for prim in prims.prims:
#     prims_dict[prim.id] = prim



while True:
    input_str = input("Enter command ")
    arduino.write(bytes(input_str + "\r\n", "utf-8"))
    completed = False
    while not completed:
        line = arduino.readline()
        print(line)
        if b"finished" in line:
            completed = True
