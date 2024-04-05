# Nano: ssh -X tnp@192.168.1.160

import serial
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


# arduino = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)


prims = get_prims("prims_8angles.txt")
prims_dict = dict()
for prim in prims.prims:
    prims_dict[prim.id] = prim

prim_id_commands = [26, 17, 17, 17, 17, 9, 28, 18, 10, 2]

prim_id_commands = get_path(
    map_size=[114, 122],
    obstacles=[[52, 96, 0, 10], [96, 114, 0, 16], [0, 63, 114, 122]],
    start=[0, 0, 0],
    goal=[88, 120, 2],
    prims="prims_8angles.txt",
)

arduino_fw_conversion = prims.resolution / 0.2
arduino_t_conversion = 1 / (78 * np.pi / 180)
cmd_sequence = ""
for id in prim_id_commands:
    prim = prims_dict[id]
    # fw command if angle doesn't change
    if prim.start_angle == prim.endpose[2]:
        if prim.endpose[0] == 0:
            dist = abs(prim.endpose[1]) * arduino_fw_conversion
        else:
            dist = abs(prim.endpose[0]) * arduino_fw_conversion
        cmd = "fw" + str(dist)

    # t command since robot is turning
    else:
        angle = (
            (prim.endpose[2] - prim.start_angle)
            * (np.pi * 2)
            / prims.num_angles
            * arduino_t_conversion
        )
        cmd = f"t{angle:.3f}"

    cmd_sequence += cmd + ":"
print(cmd_sequence)
# arduino.write(bytes(cmd_sequence + "\r\n", "utf-8"))

exit()
while True:
    input_str = input("Enter command ")
    arduino.write(bytes(input_str + "\r\n", "utf-8"))
