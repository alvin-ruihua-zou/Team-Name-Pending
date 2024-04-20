# Nano: ssh -X tnp@192.168.1.160

import serial
import time
import numpy as np
import re
from generate_graph_opt import get_path
from stair_detection import detect_stairs, check_dist


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
        # fw/bw command if angle doesn't change
        if prim.start_angle == prim.endpose[2]:
            if prim.endpose[0] == 0:
                dist = abs(prim.endpose[1]) * arduino_fw_conversion
            else:
                dist = abs(prim.endpose[0]) * arduino_fw_conversion
            # bw
            if prim.endpose[0] < 0 or prim.endpose[1] < 0:
                cmd = "bw" + str(dist)
                if prev_cmd != "bw":
                    prev_cmd = "bw"
            # fw
            else:
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
        if cmd[0] == "t":
            cmd_sequence += prev_cmd + str(fw_sum) + ":" + cmd + ":"
            fw_sum = 0
            prev_cmd = "t"
        elif cmd[:2] == "fw" and prev_cmd == "fw":
            # Append cmd if it's the last one
            if i == len(prim_id_commands) - 1:
                cmd_sequence += cmd + ":"
            else:
                fw_sum += dist
        elif cmd[:2] == "bw" and prev_cmd == "bw":
            # Append cmd if it's the last one
            if i == len(prim_id_commands) - 1:
                cmd_sequence += cmd + ":"
            else:
                fw_sum += dist
        # TODO: Switching between fw and bw

        else:
            cmd_sequence += prev_cmd + str(fw_sum) + ":"
            prev_cmd = cmd
            fw_sum = 0

    cmd_sequence = cmd_sequence.split(":")[:-1]
    cmd = cmd_sequence[0]
    if "fw" in cmd:
        rev = cmd[2:]
        print(rev)
        rev = float(rev)
        if rev < 1:
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


# Navigate from top to stairs to goal
def climbing2navigate(map, resolution=0.1, goal=[20, 4]):

    # First determine robot's position by checking distance to wall.
    # Turn right 90 degrees, check dist to wall, turn left 90 degrees
    arduino.write(bytes("t1.15:\r\n", "utf-8"))
    dist = check_dist()
    arduino.write(bytes("t-1.15:\r\n", "utf-8"))
    # Convert dist from mm to m, then to map resolution
    dist = int(dist / 1000 * resolution)
    # Assume robot is at the edge of the stairs.
    start = [map[0] - dist, 4, 1]
    curr_pos = start
    while True:
        print("before planing, curr pos is", curr_pos)
        cmd_sequence, curr_pos, complete, cmd = plan(
            start=curr_pos, goal=goal, obstacles=obstacles, map_size=map
        )
        if complete:
            break
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
                        x = str(arduino.readline())
                        x = float(re.findall("\d+\.\d+", x)[0])
                        th = str(arduino.readline())
                        th = float(re.findall("\d+\.\d+", th)[0])
                        print(x, y, th)
                        curr_pos[:2] = [start[0] + x / 25.4, start[1] + y / 25.4]
                        odom_received = True
                        break

        print("after executing, curr pos is", curr_pos)


# Navigate from start to goal then climb stairs.
def navigation2climbing(
    start=[5, 5, 1],
    goal=[88, 100, 1],
    obstacles=[[52, 96, 0, 10], [96, 114, 0, 16], [0, 63, 104, 122]],
    map_size=[114, 122],
):
    steps = 0
    temp_goal = goal
    # Define temp goal 0.5 meters away from goal.
    if goal[2] == 0:
        temp_goal[0] -= 5
    elif goal[2] == 1:
        temp_goal[1] -= 5
    elif goal[2] == 2:
        temp_goal[0] += 5
    elif goal[2] == 3:
        temp_goal[1] += 5
    curr_pos = start
    counter = 0
    max_len = 999
    while True:

        if counter >= max_len:
            if curr_pos[2] == temp_goal[2]:
                complete = True

        print("cmd_sequence len", max_len)
        print("counter is: ", counter)
        print("curr pose:", curr_pos, "goal pos:", temp_goal)
        cmd_sequence, curr_pos, complete, cmd = plan(start=curr_pos, goal=temp_goal)

        if counter == 0:
            max_len = len(cmd_sequence) + 1
            counter = 1
        else:
            counter += 1

        if complete:
            see_stairs = detect_stairs()
            if see_stairs:
                print("Stair detected, proceed")
            else:
                print("Stair not detected, aborting...")
                exit()
            break
        # has_turn = False
        # for c in cmd_sequence:
        #     if "t" in c:
        #         has_turn = True
        # if has_turn == False:
        #     see_stairs = detect_stairs()
        #     if see_stairs:
        #         print("Stair detected, proceed")
        #     else:
        #         print("Stair not detected, aborting...")
        #         exit()

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
                        x = str(arduino.readline())
                        x = float(re.findall("\d+\.\d+", x)[0])
                        th = str(arduino.readline())
                        th = float(re.findall("\d+\.\d+", th)[0])
                        print(x, y, th)
                        curr_pos[:2] = [start[0] + x / 25.4, start[1] + y / 25.4]
                        odom_received = True
                        break

        print("after executing, curr pos is", curr_pos)
    # Move from temp goal to goal
    arduino.write(bytes("fw2.5:\r\n", "utf-8"))
    print("planning complete, start stair climbing")
    input("Start climbing?")
    # Move fw2 to ensure robot is against stairs, then climb to final step, then climb final step, then move fw2
    arduino.write(bytes("fw2:i9:step:fw2:\r\n", "utf-8"))


# NSH staircase map:
# map size: [103, 50] obstacles: [[84,103,0,15]]

if __name__ == "__main__":
    time.sleep(0.5)
    mode = input("Select mode:\nNavigation + stair climbing [0]\nCommand control [1]")
    if mode.strip() == "0":
        start = input("starting pos(three numbers with space between): ")
        start = list(start.split(" "))
        start = [int(i) for i in start]
        navigation2climbing(
            start=start,
            goal=[78, 48, 1],
            obstacles=[[84, 103, 0, 15]],
            map_size=[103, 50],
        )
    elif mode.strip() == "1":
        while True:
            input_str = input("Enter command ")
            if input_str.strip() == "cam":
                print(f"Stairs detected: {detect_stairs()}")
            elif input_str.strip() == "dist":
                print(f"Distance to wall: {check_dist()}")
            else:
                arduino.write(bytes(input_str + "\r\n", "utf-8"))
                completed = False
                while not completed:
                    line = arduino.readline()
                    print(line)
                    if b"finished" in line:
                        completed = True
