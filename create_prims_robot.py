import numpy as np


def create_prim(resolution, num_angles, ID, start_angle, endpose, cost, num_poses):
    print(f"primID: {ID}")
    print(f"startangle_c: {start_angle}")
    print(f"endpose_c: {endpose[0]} {endpose[1]} {endpose[2]}")
    print(f"additionalactioncostmult: {cost}")
    print(f"intermediateposes: {num_poses}")
    angle_res = np.pi * 2 / num_angles
    x = 0.0000
    y = 0.0000
    theta = start_angle * angle_res

    for i in range(num_poses):
        print(f"{x:.4f} {y:.4f} {theta:.6f}")
        x += endpose[0] * resolution / (num_poses - 1)
        y += endpose[1] * resolution / (num_poses - 1)
        theta += (endpose[2] - start_angle) * angle_res / (num_poses - 1)


# Forward prims
# id = 0
# num_angles = 8
# dirs = [[1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0], [-1, -1], [0, -1], [1, -1]]
# for i in [1, 2, 5]:
#     for angle in range(num_angles):
#         dir = dirs[angle]
#         create_prim(
#             resolution=0.1,
#             num_angles=num_angles,
#             ID=id,
#             start_angle=angle,
#             endpose=[dir[0] * i, dir[1] * i, angle],
#             cost=1,
#             num_poses=10,
#         )
#         id += 1

# Turning prims
# id = 24
# num_angles = 8
# cost = 50
# for angle in range(num_angles):
#     if angle == 0:
#         create_prim(
#             resolution=0.1,
#             num_angles=num_angles,
#             ID=id,
#             start_angle=7,
#             endpose=[0, 0, 0],
#             cost=cost,
#             num_poses=10,
#         )
#         id += 1
#         create_prim(
#             resolution=0.1,
#             num_angles=num_angles,
#             ID=id,
#             start_angle=0,
#             endpose=[0, 0, 7],
#             cost=cost,
#             num_poses=10,
#         )
#         id += 1
#     else:
#         create_prim(
#             resolution=0.1,
#             num_angles=num_angles,
#             ID=id,
#             start_angle=angle - 1,
#             endpose=[0, 0, angle],
#             cost=cost,
#             num_poses=10,
#         )
#         id += 1
#         create_prim(
#             resolution=0.1,
#             num_angles=num_angles,
#             ID=id,
#             start_angle=angle,
#             endpose=[0, 0, angle - 1],
#             cost=cost,
#             num_poses=10,
#         )
#         id += 1
