from matplotlib import pyplot as plt
import queue
import time
import math
import numpy as np
import copy
import matplotlib.animation as animation
from matplotlib.lines import Line2D
from functools import partial
import matplotlib.patches as patches
import matplotlib as mpl


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


class Car:
    def __init__(self, width, length):
        self.width = width
        self.length = length


class environ:
    def __init__(self, env_map, rows, cols, resolution):
        self.map = env_map
        self.rows = rows
        self.cols = cols
        self.resolution = resolution


class Node:
    def __init__(self, state, node_id, h):
        self.state = state
        self.id = node_id
        self.g = 10000
        self.h = h
        self.v = 10000
        self.incons = False
        self.bp = 0
        self.inopen = False
        self.closed = False
        self.action = 0

    def __lt__(self, other):
        return self.g + self.h < other.g + other.h

    def __str__(self):
        return f"Node:\nid: {self.id}\nstate: {self.state}\n"


class Graph:
    def __init__(self, nodes):
        self.nodes = nodes
        self.num_nodes = len(nodes)
        self.state_set = set()
        self.state_dict = dict()

    def addNode(self, node):
        self.nodes.append(node)
        self.num_nodes += 1
        self.state_set.add(tuple(node.state))
        self.state_dict[tuple(node.state)] = node.id

    def print_edges(self):
        for i in range(self.num_nodes):
            if i in self.edges:
                print(f"Node: {i} ", end=" ")
                print(f"edges: {self.edges[i]}")


def get_hval(state, goal):
    return abs(state[0] - goal[0]) + abs(state[1] - goal[1])  # +abs(state[2]-goal[2]))


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


def visualize_prims(prims):
    for prim in prims.prims:
        xx = [s[0] for s in prim.inter_poses]
        xy = [s[1] for s in prim.inter_poses]
        # plt.plot(xx, xy)
        # plt.gca().set_aspect("equal")
        # plt.show()
        # plt.savefig("prims.png")


def state_exists(graph, new_pos):
    if tuple(new_pos) in graph.state_set:
        return True
    return False


def get_node_id(graph, pos):
    return graph.state_dict[tuple(pos)]


def check_collision(state, prim, car, env):
    """
    car_r = math.sqrt(car.length**2+car.width**2)
    tiles_in_range = round(car_r/env.resolution)
    for i in range(prim.num_interposes):

      x = state[0]+prim.inter_poses[i][0]/env.resolution
      y = state[1]+prim.inter_poses[i][1]/env.resolution
      row_c = round(y)
      col_c = round(x)
      for row in range(-tiles_in_range,tiles_in_range):
        for col in range(-tiles_in_range,tiles_in_range):
          if 0 <= row_c+row and row_c+row < env.rows and 0 <= col_c+col and col_c+col < env.cols and env.map[row_c+row][col_c+col] == 1:
            #print(f"state {state} with prim {prim.endpose} collides with {row_c+row},{col_c+col}")
            return False
    #print(f"state {state} with prim {prim.endpose} is collision free")
    return True
    """
    row_c = state[1]
    col_c = state[0]
    for cell in prim.cells_covered:
        if (
            0 <= row_c + cell[1]
            and row_c + cell[1] < env.rows
            and 0 <= col_c + cell[0]
            and col_c + cell[0] < env.cols
            and env.map[row_c + int(cell[1])][col_c + int(cell[0])] == 1
        ):
            # print(f"state {state} with prim {prim.endpose} collides with {row_c+row},{col_c+col}")
            return False
    # print(f"state {state} with prim {prim.endpose} is collision free")
    return True


def backward_astar(env, goal_state):
    graph = Graph(list())
    curr_id = 1
    epsilon = 1
    open_q = queue.PriorityQueue()
    goal_node = Node(goal_state, 0, 0)
    graph.addNode(goal_node)
    goal_node.g = 0
    graph.nodes[goal_node.id].g = 0
    goal_node.inopen = True
    graph.nodes[goal_node.id].inopen = True
    open_q.put(goal_node)

    while not open_q.empty():
        curr_node = open_q.get()
        curr_node.closed = True
        graph.nodes[curr_node.id].closed = True
        # print(f'curr_node: {curr_node.id}, {curr_node.state}')

        # Use high resolution prim if within some radius of goal
        rad = -1

        for dir_off in [
            [-1, -1],
            [-1, 0],
            [-1, 1],
            [1, 1],
            [1, -1],
            [1, 0],
            [0, 1],
            [0, -1],
        ]:
            new_pos = [curr_node.state[0] + dir_off[0], curr_node.state[1] + dir_off[1]]
            # Check if new state is in bounds
            if (
                0 <= new_pos[0]
                and new_pos[0] < env.cols
                and 0 <= new_pos[1]
                and new_pos[1] < env.rows
                and env.map[new_pos[1]][new_pos[0]] == 0
            ):

                # Check if the state already exists
                if not state_exists(graph, new_pos):

                    new_node = Node(new_pos, curr_id, 0)
                    curr_id += 1
                    graph.addNode(new_node)
                else:
                    new_node = graph.nodes[get_node_id(graph, new_pos)]

                # graph.addEdge(curr_node, new_node, prim)

                if not new_node.closed and new_node.g > curr_node.g + 1:
                    new_node.g = curr_node.g + 1
                    graph.nodes[new_node.id].g = curr_node.g + 1
                    graph.nodes[new_node.id].bp = curr_node.id

                    if not new_node.inopen:
                        # if new_pos == [7,0,1]: print(new_node.id,new_node.state,graph.nodes[new_node.id].state, graph.nodes[new_node.id].bp, curr_node.id,curr_node.state)
                        new_node.inopen = True
                        graph.nodes[new_node.id].inopen = True
                        open_q.put(new_node)

    return graph


def generate_graph(prims, prims_highres, env, start_state, goal_state, car, p_map):
    graph = Graph(list())
    curr_id = 1
    epsilon = 10
    open_q = queue.PriorityQueue()

    start_t = time.time()
    hval_graph = backward_astar(env, goal_state[:2])
    end_t = time.time()
    print(
        f"backward A* complete with {end_t-start_t} seconds, {hval_graph.num_nodes} nodes\n"
    )

    start_node = Node(start_state, 0, 0)
    graph.addNode(start_node)
    start_node.g = 0
    graph.nodes[start_node.id].g = 0
    start_node.inopen = True
    graph.nodes[start_node.id].inopen = True
    open_q.put(start_node)

    while not open_q.empty() and (
        not state_exists(graph, goal_state)
        or not graph.nodes[get_node_id(graph, goal_state)].closed
    ):
        curr_node = open_q.get()
        curr_node.closed = True
        graph.nodes[curr_node.id].closed = True
        # print(f'curr_node: {curr_node.id}, {curr_node.state}')

        # Use high resolution prim if within some radius of goal
        rad = -1
        if np.sum(np.abs(np.array(curr_node.state) - np.array(goal_state))) < rad:
            for prim in prims_highres.prims:
                # Check if prim has matching angle
                if prim.start_angle == curr_node.state[2]:
                    new_pos = [
                        curr_node.state[0] + prim.endpose[0],
                        curr_node.state[1] + prim.endpose[1],
                        prim.endpose[2],
                    ]
                    # Check if new state is in bounds
                    if (
                        0 <= new_pos[0]
                        and new_pos[0] < env.cols
                        and 0 <= new_pos[1]
                        and new_pos[1] < env.rows
                        and collision_precheck(new_pos, prim, env, p_map)
                        and check_collision(curr_node.state, prim, car, env)
                    ):

                        # Check if the state already exists
                        if not state_exists(graph, new_pos):

                            new_node = Node(
                                new_pos,
                                curr_id,
                                hval_graph.nodes[get_node_id(hval_graph, new_pos[:2])].g
                                * epsilon,
                            )
                            # new_node = Node(new_pos,curr_id,get_hval(new_pos,goal_state)*epsilon)
                            curr_id += 1
                            graph.addNode(new_node)
                        else:
                            new_node = graph.nodes[get_node_id(graph, new_pos)]

                        # graph.addEdge(curr_node, new_node, prim)

                        if not new_node.closed and new_node.g > curr_node.g + prim.cost:
                            new_node.g = curr_node.g + prim.cost
                            graph.nodes[new_node.id].g = curr_node.g + prim.cost
                            graph.nodes[new_node.id].bp = curr_node.id
                            graph.nodes[new_node.id].action = prim

                            if not new_node.inopen:
                                # if new_pos == [7,0,1]: print(new_node.id,new_node.state,graph.nodes[new_node.id].state, graph.nodes[new_node.id].bp, curr_node.id,curr_node.state)
                                new_node.inopen = True
                                graph.nodes[new_node.id].inopen = True
                                open_q.put(new_node)
        else:
            for prim in prims.prims:
                # Check if prim has matching angle
                if prim.start_angle == curr_node.state[2]:
                    new_pos = [
                        curr_node.state[0] + prim.endpose[0],
                        curr_node.state[1] + prim.endpose[1],
                        (prim.endpose[2] + 16) % 16,
                    ]

                    # Check if new state is in bounds
                    if (
                        0 <= new_pos[0]
                        and new_pos[0] < env.cols
                        and 0 <= new_pos[1]
                        and new_pos[1] < env.rows
                        and collision_precheck(new_pos, prim, env, p_map)
                        and check_collision(curr_node.state, prim, car, env)
                    ):

                        # Check if the state already exists
                        if not state_exists(graph, new_pos):

                            new_node = Node(
                                new_pos,
                                curr_id,
                                hval_graph.nodes[get_node_id(hval_graph, new_pos[:2])].g
                                * epsilon,
                            )
                            # new_node = Node(new_pos,curr_id,get_hval(new_pos,goal_state)*epsilon)
                            curr_id += 1
                            graph.addNode(new_node)
                        else:
                            new_node = graph.nodes[get_node_id(graph, new_pos)]

                        # graph.addEdge(curr_node, new_node, prim)

                        if not new_node.closed and new_node.g > curr_node.g + prim.cost:
                            new_node.g = curr_node.g + prim.cost
                            graph.nodes[new_node.id].g = curr_node.g + prim.cost
                            graph.nodes[new_node.id].bp = curr_node.id
                            graph.nodes[new_node.id].action = prim

                            if not new_node.inopen:
                                # if new_pos == [7,0,1]: print(new_node.id,new_node.state,graph.nodes[new_node.id].state, graph.nodes[new_node.id].bp, curr_node.id,curr_node.state)
                                new_node.inopen = True
                                graph.nodes[new_node.id].inopen = True
                                open_q.put(new_node)
    #print(graph.nodes[-1], new_node, goal_state)
    return graph


def generate_graph_AD(
    prims,
    epsilon,
    open_init,
    id_init,
    hval_graph,
    env,
    start_state,
    goal_state,
    car,
    p_map,
):
    graph = Graph(list())
    curr_id = id_init
    open_q = queue.PriorityQueue()
    for node in open_init:
        open_q.put(node)

    print(
        f"backward A* complete with {end_t-start_t} seconds, {hval_graph.num_nodes} nodes\n"
    )

    start_node = Node(start_state, 0, 0)
    graph.addNode(start_node)
    start_node.g = 0
    graph.nodes[start_node.id].g = 0
    start_node.inopen = True
    graph.nodes[start_node.id].inopen = True
    open_q.put(start_node)

    while not open_q.empty():
        curr_node = open_q.get()
        if (
            state_exists(graph, goal_state)
            and (
                graph.nodes[get_node_id(graph, goal_state)].g
                + graph.nodes[get_node_id(graph, goal_state)].h
            )
            > curr_node.g + curr_node.h
        ):
            break
        curr_node.closed = True
        graph.nodes[curr_node.id].closed = True

        # print(f'curr_node: {curr_node.id}, {curr_node.state}')

        # Use high resolution prim if within some radius of goal

        for prim in prims.prims:
            # Check if prim has matching angle
            if prim.start_angle == curr_node.state[2]:
                new_pos = [
                    curr_node.state[0] + prim.endpose[0],
                    curr_node.state[1] + prim.endpose[1],
                    (prim.endpose[2] + 16) % 16,
                ]
                # Check if new state is in bounds
                if (
                    0 <= new_pos[0]
                    and new_pos[0] < env.cols
                    and 0 <= new_pos[1]
                    and new_pos[1] < env.rows
                    and collision_precheck(new_pos, prim, env, p_map)
                    and check_collision(curr_node.state, prim, car, env)
                ):

                    # Check if the state already exists
                    if not state_exists(graph, new_pos):

                        new_node = Node(
                            new_pos,
                            curr_id,
                            hval_graph.nodes[get_node_id(hval_graph, new_pos[:2])].g
                            * epsilon,
                        )
                        # new_node = Node(new_pos,curr_id,get_hval(new_pos,goal_state)*epsilon)
                        curr_id += 1
                        graph.addNode(new_node)
                    else:
                        new_node = graph.nodes[get_node_id(graph, new_pos)]

                    # graph.addEdge(curr_node, new_node, prim)

                    if new_node.g > curr_node.g + prim.cost:
                        new_node.g = curr_node.g + prim.cost
                        graph.nodes[new_node.id].g = curr_node.g + prim.cost
                        graph.nodes[new_node.id].bp = curr_node.id
                        graph.nodes[new_node.id].action = prim

                        if not new_node.closed:
                            # if new_pos == [7,0,1]: print(new_node.id,new_node.state,graph.nodes[new_node.id].state, graph.nodes[new_node.id].bp, curr_node.id,curr_node.state)
                            new_node.inopen = True
                            graph.nodes[new_node.id].inopen = True
                            open_q.put(new_node)
                        else:
                            new_node.incons = True
                            graph.nodes[new_node.id].incons = True

    over_cons = list()
    curr_id = 0
    for node in graph.nodes:
        if node.incons or node.inopen:
            node.id = curr_id
            curr_id += 1
            over_cons.append(node)
    return graph, over_cons, curr_id


def load_env(map_file, resolution):
    f = open(map_file, "r")
    lines = f.readlines()
    rows = 0
    cols = len(lines[0].split())
    env_map = list()
    for line in lines:
        rows += 1
        env_map.append([int(n) for n in line.split()])
    f.close()
    return environ(env_map, rows, cols, resolution)


def search_graph(graph, start_state, goal_state):
    start_found = False
    goal_found = False
    for node in graph.nodes:
        if start_state == node.state:
            start_node = node
            start_found = True
        if goal_state == node.state:
            goal_node = node
            goal_found = True
        if start_found and goal_found:
            break
    if not start_found or not goal_found:
        print(f"Error: invalid states {start_state} {goal_state}")
        return list()

    # Get path
    if start_state == goal_state:
        return list()
    start_found = False
    curr_node = goal_node
    actions = list()
    while not start_found:
        # print(curr_node.id,curr_node.state,curr_node.bp)
        actions.append(curr_node.action)
        curr_node = graph.nodes[curr_node.bp]
        if curr_node.id == start_node.id:
            start_found = True
            # print(curr_node.id,curr_node.state)
    return reversed(actions)


def AD(prims, env, start_state, goal_state, car, p_map):
    start_found = False
    goal_found = False
    hval_graph = backward_astar(env, goal_state[:2])
    open_init, id_init = list(), 1
    epsilons = [100, 20, 10, 5, 2, 1]
    for epsilon in epsilons:
        graph, over_cons, curr_id = generate_graph_AD(
            prims,
            epsilon,
            open_init,
            id_init,
            hval_graph,
            env,
            start_state,
            goal_state,
            car,
            p_map,
        )
        open_init, id_init = over_cons, curr_id
        for node in graph.nodes:
            if start_state == node.state:
                start_node = node
                start_found = True
            if goal_state == node.state:
                goal_node = node
                goal_found = True
            if start_found and goal_found:
                break
        if not start_found or not goal_found:
            print("Error: invalid states")

        # Get path
        start_found = False
        curr_node = goal_node
        actions = list()
        while not start_found:
            # print(curr_node.id,curr_node.state,curr_node.bp)
            actions.append(curr_node.action)
            curr_node = graph.nodes[curr_node.bp]
            if curr_node.id == start_node.id:
                start_found = True
                # print(curr_node.id,curr_node.state)
        actions = reversed(actions)


def create_map(size, outfile, obstacles):
    # Obstacle = [xlow,xhigh,ylow,yhigh]
    x, y = size
    f = open(outfile, "w")
    for row in range(y):
        for col in range(x - 1):
            inObs = False
            for obstacle in obstacles:
                if (
                    obstacle[0] <= col < obstacle[1]
                    and obstacle[2] <= row < obstacle[3]
                ):
                    f.write("1 ")
                    inObs = True
                    break
            if not inObs:
                f.write("0 ")
        inObs = False
        for obstacle in obstacles:
            if obstacle[0] <= col < obstacle[1] and obstacle[2] <= row < obstacle[3]:
                f.write("1\n")
                inObs = True
                break
        if not inObs:
            f.write("0\n")
    f.close()


def create_pmap(env, car):
    p_map = copy.deepcopy(env.map)
    car_r = math.sqrt(car.length**2 + car.width**2)
    tiles_in_range = round(car_r / env.resolution)
    for row in range(env.rows):
        for col in range(env.cols):
            if env.map[row][col] == 1:
                for row_off in range(tiles_in_range):
                    for col_off in range(tiles_in_range):
                        if (
                            0 <= row + row_off
                            and row + row_off < env.rows
                            and 0 <= col + col_off
                            and col + col_off < env.cols
                        ):
                            p_map[row + row_off][col + col_off] = 1
    return p_map


def get_cells_covered(x, car, resolution):
    l = car.length
    w = car.width
    theta = np.arctan(w / l)
    r = np.sqrt((l / 2) ** 2 + (w / 2) ** 2)
    points = [
        [x[0] + l / 2 * np.cos(x[2] - theta), x[1] + l / 2 * np.sin(x[2] - theta)],
        [x[0] - l / 2 * np.cos(x[2] - theta), x[1] - l / 2 * np.sin(x[2] - theta)],
        [x[0] + l / 2 * np.cos(x[2] + theta), x[1] + l / 2 * np.sin(x[2] + theta)],
        [x[0] - l / 2 * np.cos(x[2] + theta), x[1] - l / 2 * np.sin(x[2] + theta)],
        [x[0] + l / 2 * np.cos(x[2]), x[1] + l / 2 * np.sin(x[2])],
        [x[0] - l / 2 * np.cos(x[2]), x[1] - l / 2 * np.sin(x[2])],
        [
            x[0] + w / 2 * np.cos(np.pi / 2 - x[2]),
            x[1] - w / 2 * np.sin(np.pi / 2 - x[2]),
        ],
        [
            x[0] - w / 2 * np.cos(np.pi / 2 - x[2]),
            x[1] + w / 2 * np.sin(np.pi / 2 - x[2]),
        ],
    ]
    """
  [x[0]+l/2*np.cos(x[2]),x[1]+l/2*np.sin(x[2])],
  [x[0]-l/2*np.cos(x[2]),x[1]-l/2*np.sin(x[2])],
  [x[0]+w/2*np.cos(np.pi/2-x[2]),x[1]-w/2*np.sin(np.pi/2-x[2])],
  [x[0]-w/2*np.cos(np.pi/2-x[2]),x[1]+w/2*np.sin(np.pi/2-x[2])]
  """
    res = set()
    for p in points:
        res.add((round(p[0] / resolution), round(p[0] / resolution)))
    return res


def cal_prim_cells_covered(prims, car):
    for prim in prims.prims:
        cells_covered = set()
        for pose in prim.inter_poses:
            car_r = math.sqrt(car.length**2 + car.width**2)
            tiles_in_range = round(car_r / prims.resolution)
            x = pose[0] / prims.resolution
            y = pose[1] / prims.resolution
            row_c = round(y)
            col_c = round(x)
            c_temp = get_cells_covered(pose, car, prims.resolution)
            cells_covered = cells_covered.union(c_temp)
            # for row in range(-tiles_in_range,tiles_in_range):
            #  for col in range(-tiles_in_range,tiles_in_range):
            #    cells_covered.add((col_c+col,row_c+row))
        prim.cells_covered = cells_covered


def collision_precheck(state, prim, env, p_map):
    for i in range(prim.num_interposes):
        x = state[0] + prim.inter_poses[i][0] / env.resolution
        y = state[1] + prim.inter_poses[i][1] / env.resolution
        row_c = int(round(y))
        col_c = int(round(x))
        if (
            0 <= row_c
            and row_c < env.rows
            and 0 <= col_c
            and col_c < env.cols
            and p_map[row_c][col_c] == 1
        ):
            return False
    return True


def save_traj(traj_x, traj_y, traj_theta, filename):
    f = open(filename, "w")
    for i in range(len(traj_x)):
        f.write(f"{traj_x[i]} {traj_y[i]} {traj_theta[i]}\n")
    f.close()


def animate(n, x, y, theta):
    line.set_xdata(x[:n])
    line.set_ydata(y[:n])
    patch.set_width(car.width)
    patch.set_height(car.length)
    patch.set_xy([x[n] - car.width / 2, y[n] - car.length / 2])

    tr = mpl.transforms.Affine2D().rotate_deg_around(x[n], y[n], -np.rad2deg(theta[n]))
    t = tr + ts
    patch.set_transform(t)
    # patch.set_angle = -30#-np.rad2deg(theta[n])
    return (
        line,
        patch,
    )


def inches_to_resolution(inches, res):
    return inches * 2.54 / 100 / res


# start and goal are entered as inches
def get_path(map_size, obstacles, start, goal, prims="prims_8angles.txt"):
    start_prim = time.time()
    prims = get_prims(prims)
    prims_highres = prims
    end_prim = time.time()

    # obstacles: [x1,x2,y1,y2], origin bottom left
    obstacles = np.array(obstacles)
    obstacles = inches_to_resolution(obstacles, prims.resolution)

    create_map(
        np.round(inches_to_resolution(np.array(map_size), prims.resolution)).astype(
            int
        ),
        "map.txt",
        obstacles,
    )
    car = Car(0.1, 0.1)
    start_env = time.time()
    env = load_env("map.txt", prims.resolution)

    cal_prim_cells_covered(prims, car)
    p_map = create_pmap(env, car)

    end_env = time.time()
    start = [
        np.round(inches_to_resolution(start[0], env.resolution)).astype(int),
        np.round(inches_to_resolution(start[1], env.resolution)).astype(int),
        start[2],
    ]

    goal = [
        np.round(inches_to_resolution(goal[0], env.resolution)).astype(int),
        np.round(inches_to_resolution(goal[1], env.resolution)).astype(int),
        goal[2],
    ]

    start_time = time.time()
    graph = generate_graph(prims, prims_highres, env, start, goal, car, p_map)

    actions = search_graph(graph, start, goal)
    end_time = time.time()
    print(f"Time to find plan: {start_time - end_time} seconds")
    print(f"States expanded: {graph.num_nodes}")

    traj_x = list()
    traj_y = list()
    traj_theta = list()
    state = np.array(start) * env.resolution
    prim_ids = []
    for action in actions:
        prim_ids.append(action.id)
        for s in action.inter_poses:
            traj_x.append(s[0] + state[0])
            traj_y.append(s[1] + state[1])
            traj_theta.append((s[2]) % (2 * math.pi))

        state = state + np.array(action.inter_poses[-1])
        state[2] = action.inter_poses[-1][-1]

    save_traj(traj_x, traj_y, traj_theta, "traj.txt")

    disp_map = []
    for row in range(len(env.map)):
        arr = np.array(env.map[row])
        disp_map.append(arr.tolist())
    plt.imshow(disp_map, cmap="gray_r", origin="lower")
    plt.plot(np.array(traj_x) / prims.resolution, np.array(traj_y) / prims.resolution)
    plt.gca().set_aspect("equal")

    plt.savefig("traj_imposed.png")

    print(f"Time to load prim: {end_prim-start_prim} seconds")
    print(f"Time to load env: {end_env-start_env} seconds")
    print(f"Time to find plan: {end_time-start_time} seconds")
    print(f"States expanded: {graph.num_nodes}")
    return prim_ids


if __name__ == "__main__":

    start_prim = time.time()
    prims = get_prims("prims_8angles.txt")
    prims_highres = prims
    # prims_highres = get_prims("prim_highres.txt")  # unicycle_prims.mprim
    end_prim = time.time()
    # for prim in prims.prims:
    # print(f"startpose: {prim.inter_poses[0]}endpose: {prim.endpose}\n")
    # exit()

    # obstacles = [[440,442,60,84],[440,460,60,62],[440,460,82,84],[100,120,0,50],[200,210,30,100],[250,260,0,70],[300,310,30,100]]
    # obstacles: [x1,x2,y1,y2], origin bottom left
    obstacles = [[52, 96, 0, 10], [96, 114, 0, 16], [0, 63, 114, 122]]
    # obstacles = [[20,25,0,200],[25,80,190,200],[95,100,100,250],[50,100,150,155],
    #              [50,200,50,70],[230,450,30,170],[350,355,280,325],[130,370,325,330],[60,400,450,460],[200,500,360,370],[140,350,280,290],[130,140,70,300],[50,100,230,240],[60,70,240,480],[200,500,200,210],[140,150,0,50]]
    obstacles = np.array(obstacles)
    obstacles = inches_to_resolution(obstacles, prims.resolution)
    # obstacles = np.vstack([obstacles, obstacles+500])

    # create_map([500,500],'map_big.txt',obstacles)

    create_map(
        np.round(inches_to_resolution(np.array([114, 122]), prims.resolution)).astype(
            int
        ),
        "map_big.txt",
        obstacles,
    )
    car = Car(0.1, 0.1)
    start_env = time.time()
    env = load_env("map_big.txt", prims.resolution)

    cal_prim_cells_covered(prims, car)
    p_map = create_pmap(env, car)

    end_env = time.time()
    start = [0, 0, 0]
    # goal = [160,20,8]
    # goal = [480,100,0]
    # goal = [300,480,14]
    goal = [
        np.round(inches_to_resolution(88, env.resolution)).astype(int),
        np.round(inches_to_resolution(120, env.resolution)).astype(int),
        2,
    ]
    print(
        goal,
        np.round(inches_to_resolution(np.array([114, 122]), prims.resolution)).astype(
            int
        ),
    )
    avg_t = 0
    avg_n = 0
    iters = 1
    for i in range(iters):
        start_time = time.time()
        graph = generate_graph(prims, prims_highres, env, start, goal, car, p_map)
        # print(f'Num_nodes: {graph.num_nodes}')
        # for i in range(10):
        #  print(i,graph.nodes[i].state,graph.nodes[i].bp)
        actions = search_graph(graph, start, goal)
        end_time = time.time()
        avg_t += start_time - end_time
        avg_n += graph.num_nodes
    print(f"Time to find plan: {avg_t/iters} seconds")
    print(f"States expanded: {avg_n/iters}")

    traj_x = list()
    traj_y = list()
    traj_theta = list()
    state = np.array(start) * env.resolution
    for action in actions:
        # print(action.id,action.endpose)
        print(action.id)
        for s in action.inter_poses:
            traj_x.append(s[0] + state[0])
            traj_y.append(s[1] + state[1])
            traj_theta.append((s[2]) % (2 * math.pi))

        state = state + np.array(action.inter_poses[-1])
        state[2] = action.inter_poses[-1][-1]
    # graph.print_edges()
    # visualize_prims(prims)

    save_traj(traj_x, traj_y, traj_theta, "traj.txt")

    # plt.plot(traj_x,np.array(traj_y))
    # plt.gca().set_aspect('equal')
    disp_map = []
    for row in range(len(env.map)):
        arr = np.array(env.map[row])
        # arr = np.repeat(arr, int(10))
        disp_map.append(arr.tolist())
    print(traj_x[-1], traj_y[-1])
    plt.imshow(disp_map, cmap="gray_r", origin="lower")
    plt.plot(np.array(traj_x) / prims.resolution, np.array(traj_y) / prims.resolution)
    plt.gca().set_aspect("equal")
    # plt.xlim([0,env.cols])
    # plt.xlim([0,env.rows])
    plt.savefig("traj_imposed.png")
    # print(traj_x )
    # print(traj_theta)
    print(f"Time to load prim: {end_prim-start_prim} seconds")
    print(f"Time to load env: {end_env-start_env} seconds")
    print(f"Time to find plan: {end_time-start_time} seconds")
    print(f"States expanded: {graph.num_nodes}")

    exit()

    fig = plt.figure()

    ax = plt.axes()
    # turn off axis spines
    # ax.xaxis.set_visible(False)
    # ax.yaxis.set_visible(False)
    # ax.set_frame_on(False)
    ax.imshow(env.map, cmap="gray_r")
    # set figure background opacity (alpha) to 0
    # fig.patch.set_alpha(0.0)
    patch = patches.Rectangle((0, 0), 0, 0, fc="red", visible=False)
    ax.add_patch(patch)
    (line,) = ax.plot([], [], lw=2)
    ts = ax.transData
    traj_x = np.array(traj_x)
    traj_y = np.array(traj_y)
    anim = animation.FuncAnimation(
        fig,
        partial(
            animate,
            x=traj_x / prims.resolution,
            y=traj_y / prims.resolution,
            theta=traj_theta,
        ),
        frames=len(traj_x),
        blit=True,
        interval=10,
    )
    anim.save("animation.gif")
