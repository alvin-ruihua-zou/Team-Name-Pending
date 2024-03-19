## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
from matplotlib import path

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == "RGB Camera":
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

colorizer = rs.colorizer()
colorizer.set_option(rs.option.color_scheme, 0)

# Start streaming
pipeline.start(config)


# Group the numbers close to each other based on a certain threshold, then return the mean of each group
def group(a, thr):
    x = np.sort(a, axis=0)
    diff = x[1:, 0] - x[:-1, 0]
    gps = np.concatenate([[0], np.cumsum(diff >= thr)])
    return [list(np.mean(x[gps == i], axis=0)) for i in range(gps[-1] + 1)]


def group_1d(a, thr):
    x = np.sort(a, axis=0)
    diff = x[1:] - x[:-1]
    gps = np.concatenate([[0], np.cumsum(diff >= thr)])
    return [list(np.mean(x[gps == i], axis=0)) for i in range(gps[-1] + 1)]


def intersection(line1, line2):
    """Finds the intersection of two lines given in Hesse normal form.

    Returns closest integer pixel locations.
    See https://stackoverflow.com/a/383527/5087436
    """
    rho1, theta1 = line1
    rho2, theta2 = line2
    A = np.array([[np.cos(theta1), np.sin(theta1)], [np.cos(theta2), np.sin(theta2)]])
    b = np.array([[rho1], [rho2]])
    x0, y0 = np.linalg.solve(A, b)
    x0, y0 = int(np.round(x0)), int(np.round(y0))
    return [x0, y0]


try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        # depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        depth_image = np.asanyarray(colorizer.colorize(depth_frame).get_data())

        # Convert to graycsale
        img_gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        # Blur the image for better edge detection
        img_blur = cv2.GaussianBlur(img_gray, (5, 5), 0)
        edges_image = cv2.Canny(image=img_blur, threshold1=10, threshold2=20)
        lines = cv2.HoughLines(edges_image, 1, np.pi / 180, 120, None, 0, 0)
        lines_image = cv2.cvtColor(edges_image, cv2.COLOR_GRAY2BGR)
        all_lines_image = lines_image.copy()
        if lines is not None:
            for i in range(len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
                pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
                cv2.line(all_lines_image, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)

        # Convert to graycsale
        # depth_gray = cv2.cvtColor(depth_image, cv2.COLOR_BGR2GRAY)
        # Blur the image for better edge detection
        # depth_blur = cv2.GaussianBlur(depth_image, (5, 5), 0)
        # depth_edges_image = cv2.Canny(image=depth_blur, threshold1=30, threshold2=40)
        # depth_lines = cv2.HoughLines(depth_edges_image, 1, np.pi / 180, 90, None, 0, 0)
        # depth_lines_image = cv2.cvtColor(depth_edges_image, cv2.COLOR_GRAY2BGR)

        # contour_image = color_image.copy()
        # contours, _ = cv2.findContours(
        #     edges_image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        # )
        # for contour in contours:
        #     area = cv2.contourArea(contour)
        #     if area > 50:
        #         epsilon = 0.04 * cv2.arcLength(contour, True)
        #         approx = cv2.approxPolyDP(contour, epsilon, True)
        #         cv2.drawContours(contour_image, [approx], 0, (0, 255, 0), 2)
        # If depth and color resolutions are different, resize color image to match depth image for display
        # if depth_colormap_dim != color_colormap_dim:
        #     resized_color_image = cv2.resize(
        #         color_image,
        #         dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
        #         interpolation=cv2.INTER_AREA,
        #     )
        #     images = np.hstack((resized_color_image, depth_colormap))
        # else:
        #     images = np.hstack((color_image, depth_colormap))

        tolerance = np.pi / 10
        threshold = 0
        lines_trimmed = []
        vert_lines = []
        vert_lines_left = []
        vert_lines_right = []
        vert_rhos_left = -10000
        vert_rhos_right = 10000
        vert_line_left = []
        vert_line_right = []
        hori_rhos = []
        if lines is not None:
            lines = lines.squeeze()
        if lines is not None and lines.ndim == 2:

            for i in range(0, len(lines)):
                rho = lines[i][0]
                theta = lines[i][1]
                # Get stair vertical lines
                # print(vert_lines)
                # print(lines_trimmed)

                if (
                    theta >= np.pi * 7 / 8 - tolerance
                    and theta <= np.pi * 7 / 8 + tolerance
                ) and (vert_rhos_left - rho < -threshold):
                    vert_lines_left.append(lines[i])
                    vert_rhos_left = rho
                    vert_line_left = lines[i]

                if theta >= np.pi / 8 - tolerance and theta <= np.pi / 8 + tolerance:
                    if vert_rhos_right - rho > threshold:
                        vert_lines_right.append(lines[i])
                        vert_rhos_right = rho
                        vert_line_right = lines[i]
                    # if vert_rhos_left - rho < -threshold:
                    #     vert_lines_left.append(lines[i])
                    #     vert_rhos_left = rho
                    #     vert_line_left = lines[i]

                # Check if the line is mostly parallel to the ground
                if (
                    theta >= np.pi / 2 - tolerance
                    and theta <= np.pi / 2 + tolerance
                    and (
                        len(hori_rhos) == 0
                        or np.all(np.abs(np.array(hori_rhos) - rho) > threshold)
                    )
                ):
                    lines_trimmed.append(lines[i])
                    hori_rhos.append(rho)
                    # a = np.cos(theta)
                    # b = np.sin(theta)
                    # x0 = a * rho
                    # y0 = b * rho
                    # pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
                    # pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
                    # cv2.line(lines_image, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)
            # lines_trimmed = np.sort(lines_trimmed, axis=0)
            lines_trimmed = np.array(lines_trimmed)
            vert_lines = vert_lines_left + vert_lines_right
            vert_lines = np.array(vert_lines)

            if vert_lines.ndim == 2:
                # vert_lines = np.array(group(vert_lines, threshold))
                # for i in range(len(vert_lines)):
                #     rho = vert_lines[i][0]
                #     theta = vert_lines[i][1]
                #     a = np.cos(theta)
                #     b = np.sin(theta)
                #     x0 = a * rho
                #     y0 = b * rho
                #     pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
                #     pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
                #     cv2.line(lines_image, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)
                if len(vert_line_left) == 2:
                    rho, theta = vert_line_left
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
                    pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
                    cv2.line(lines_image, pt1, pt2, (255, 0, 0), 7, cv2.LINE_AA)
                if len(vert_line_right) == 2:
                    rho, theta = vert_line_right
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
                    pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
                    cv2.line(lines_image, pt1, pt2, (255, 0, 0), 7, cv2.LINE_AA)

            if lines_trimmed.ndim == 2:
                # group the lines close to each other together.
                threshold = 15
                # print(lines_trimmed)

                rhos = np.array(group(lines_trimmed, threshold))
                rhos = np.sort(rhos[:, 0])
                lines_dist = (rhos - np.roll(rhos, 1))[1:]
                # print(lines_dist)
                m = np.mean(lines_dist)
                residual = np.abs(lines_dist - m)
                # print(residual)
                # print(np.count_nonzero(residual < 30) >= 4)

                # Calculate average depth value of regions. Rhos ascends from low to high
                h = depth_image.shape[0]
                w = depth_image.shape[1]

                for i in range(len(lines_trimmed)):

                    rho = lines_trimmed[i][0]
                    theta = lines_trimmed[i][1]
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
                    pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
                    if rho == np.max(rhos) or rho == np.min(rhos):
                        cv2.line(lines_image, pt1, pt2, (255, 0, 0), 3, cv2.LINE_AA)
                    else:
                        cv2.line(lines_image, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)

                # TODO: Fix bounding box stuff

                vline_l_exist = len(vert_line_left) == 2
                vline_r_exist = len(vert_line_right) == 2
                if not vline_l_exist:
                    vert_line_left = [-w / 2, np.pi]
                if not vline_r_exist:
                    vert_line_right = [w / 2, 0]
                rho_max = np.max(rhos)
                rho_min = np.min(rhos)
                if rho_max == rho_min:
                    if rho_max >= h / 2:
                        rho_min = 0
                    else:
                        rho_max = h
                intersections = []
                intersections.append(intersection(vert_line_left, [rho_max, np.pi / 2]))
                intersections.append(
                    intersection(vert_line_right, [rho_max, np.pi / 2])
                )
                intersections.append(
                    intersection(vert_line_right, [rho_min, np.pi / 2])
                )
                intersections.append(intersection(vert_line_left, [rho_min, np.pi / 2]))

                for point in intersections:
                    cv2.circle(lines_image, point, 7, (0, 255, 0), -1)
                # print(intersections)
                shape = path.Path(intersections)
                cols = np.arange(w)
                rows = np.arange(h)
                yv, xv = np.meshgrid(rows, cols, indexing="ij")
                flags = shape.contains_points(
                    np.hstack(
                        (xv.flatten()[:, np.newaxis], yv.flatten()[:, np.newaxis])
                    )
                )

                if np.sum(flags) > 0:
                    # print(np.sum(flags))
                    bounded_points = depth_image[flags.reshape(h, w), :]
                    bounded_points = depth_image * flags.reshape(h, w, 1)
                    # bounded_points = np.where(flags.reshape(h, w))
                    # print(bounded_points.shape)
                    depth_image = bounded_points

                mean_depth = np.zeros(len(rhos) - 1)
                for i in range(len(rhos) - 1):
                    rho = int(rhos[i])
                    rho2 = int(rhos[i + 1])
                    pixels = depth_image[h - rho2 : h - rho, :]
                    cv2.rectangle(img_blur, (0, rho), (w, rho2), (255, 0, 0), 2)
                    mean_depth[i] = np.mean(pixels)

                # print(mean_depth)
                print(
                    np.sum((mean_depth[:-1] >= mean_depth[1:]))
                    > int(len(mean_depth) * 2 / 4)
                )
        # Show images
        cv2.namedWindow("RealSense Edges", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("RealSense Edges", edges_image)
        # cv2.namedWindow("RealSense All Edges", cv2.WINDOW_AUTOSIZE)
        # cv2.imshow("RealSense All Edges", all_lines_image)
        cv2.namedWindow("RealSense lines", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("RealSense lines", lines_image)
        # cv2.namedWindow("RealSense depth_lines", cv2.WINDOW_AUTOSIZE)
        # cv2.imshow("RealSense depth_lines", depth_lines_image)
        # cv2.namedWindow("RealSense blur", cv2.WINDOW_AUTOSIZE)
        # cv2.imshow("RealSense blur", img_blur)
        # cv2.namedWindow("RealSense Depth", cv2.WINDOW_AUTOSIZE)
        # cv2.imshow("RealSense Depth", depth_colormap)
        cv2.namedWindow("RealSense Depth", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("RealSense Depth", depth_image)

        # cv2.namedWindow("RealSense Contour", cv2.WINDOW_AUTOSIZE)
        # cv2.imshow("RealSense Contour", contour_image)
        # cv2.namedWindow("RealSense Stereo", cv2.WINDOW_AUTOSIZE)
        # cv2.imshow("RealSense Stereo", color_image)
        cv2.waitKey(1)


finally:

    # Stop streaming
    pipeline.stop()
