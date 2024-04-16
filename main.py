# Nano: ssh -X tnp@192.168.1.160

import serial
import time
import numpy as np
import re
from generate_graph_opt import get_path
from stair_detection import detect_stairs


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


try:
    arduino = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)
except:
    try:
        arduino = serial.Serial("/dev/ttyUSB1", 9600, timeout=1)
    except:
        try:
            arduino = serial.Serial("COM11", 9600, timeout=1)
        except:
            arduino = serial.Serial("COM6", 9600, timeout=1)

prims = get_prims("prims_4angles.txt")
prims_dict = dict()
for prim in prims.prims:
    prims_dict[prim.id] = prim


def plan(
    start,
    goal=[88, 100, 1],
    map_size=[114, 122],
    obstacles=[[52, 96, 0, 10], [96, 114, 0, 16], [0, 63, 104, 122]],
):
    curr_pos = start
    if (
        np.linalg.norm(np.array(start[:2]) - np.array(goal[:2])) < 10
        and start[2] == goal[2]
    ):
        print("Within radius to stairs")
        return None, None, True, None
    prim_id_commands = get_path(
        map_size=map_size,
        obstacles=obstacles,
        start=start,
        goal=goal,
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
    for i in range(len(prim_id_commands)):
        id = prim_id_commands[i]
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
            # Append cmd if it's the last one
            if i == len(prim_id_commands) - 1:
                cmd_sequence += cmd + ":"
            else:
                fw_sum += dist
        else:
            cmd_sequence += "fw" + str(fw_sum) + ":" + cmd + ":"
            fw_sum = 0
            prev_cmd = "t"
    cmd_sequence = cmd_sequence.split(":")[:-1]
    cmd = cmd_sequence[0]
    if "fw" in cmd:
        rev = cmd[2:]
        print(rev)
        rev = float(rev)
        if rev < 1.2:
            if len(cmd_sequence) > 1:
                cmd = cmd_sequence[1]
                prim = prims_dict[prim_id_commands[1]]
                curr_pos = [
                    curr_pos[0] + prim.endpose[0],
                    curr_pos[1] + prim.endpose[1],
                    prim.endpose[2],
                ]
            else:
                print("Close enough to stairs")
                return None, None, True, None
    print(cmd_sequence)
    print(cmd)
    return cmd_sequence, curr_pos, False, cmd


# Navigate from start to goal then climb stairs.
def navigation2climbing(start=[5, 5, 1]):
    steps = 0
    curr_pos = start
    dx, dy, x_prev, y_prev = 0, 0, 0, 0
    while True:
        print("before planing, curr pos is", curr_pos)
        cmd_sequence, curr_pos, complete, cmd = plan(start=curr_pos)
        if complete:
            see_stairs = detect_stairs()
            if see_stairs:
                print("Stair detected, proceed")
            else:
                print("Stair not detected, aborting...")
                exit()
            break
        has_turn = False
        for c in cmd_sequence:
            if "t" in c:
                has_turn = True
        if has_turn == False:
            see_stairs = detect_stairs()
            if see_stairs:
                print("Stair detected, proceed")
            else:
                print("Stair not detected, aborting...")
                exit()

        print("executing", cmd)
        arduino.write(bytes(cmd + ":\r\n", "utf-8"))
        odom_received = False
        while not odom_received:
            line = arduino.readline()
            print(line)
            if b"complete" in line:
                arduino.write(bytes("odo:" + "\r\n", "utf-8"))
                while True:
                    line = arduino.readline()
                    if b"x, y, theta" in line:
                        y = str(arduino.readline())
                        y = float(re.findall("\d+\.\d+", y)[0])
                        dy = y - y_prev
                        y_prev = y
                        x = str(arduino.readline())
                        x = float(re.findall("\d+\.\d+", x)[0])
                        th = str(arduino.readline())
                        dx = x - x_prev
                        x_prev = x
                        th = float(re.findall("\d+\.\d+", th)[0])
                        print(x, y, th)
                        curr_pos[:2] = [start[0] + x / 25.4, start[1] + y / 25.4]
                        odom_received = True
                        break
        # steps += 1
        # if steps == 2:
        # exit()
        print("after executing, curr pos is", curr_pos)
    print("planning complete, start stair climbing")
    input("Start climbing?")
    arduino.write(bytes("fw2:i:\r\n", "utf-8"))


if __name__ == "__main__":
    time.sleep(0.5)
    mode = input("Select mode:\nNavigation + stair climbing [0]\nCommand control [1]")
    if mode.strip() == "0":
        start = input("starting pos(three numbers with space between): ")
        start = list(start.split(" "))
        start = [int(i) for i in start]
        navigation2climbing(start=start)
    elif mode.strip() == "1":
        while True:
            input_str = input("Enter command ")
            if input_str.strip() == "cam":
                print(f"Stairs detected: {detect_stairs()}")
            else:
                arduino.write(bytes(input_str + "\r\n", "utf-8"))
                completed = False
                while not completed:
                    line = arduino.readline()
                    print(line)
                    if b"finished" in line:
                        completed = True
