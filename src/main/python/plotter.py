# Copyright (c) 2023 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import math
import time

import matplotlib.animation
import matplotlib.pyplot as plt
import matplotlib.patches as patches


def plot(result, config):
    # Get animation points
    dt = result[0] / (len(result[1]) - 1)
    animation_points = []
    current_time = 0
    for i in range(len(result[1])):
        animation_points.append((current_time, result[1][i], result[2][i]))
        current_time += dt

    # Set up plot
    fig, ax = plt.subplots()
    start_time = time.time()

    # Animate function
    def animate(i):
        current_time = (time.time() - start_time) % (result[0] + 2) - 1
        next_index = 0
        while (
            next_index < len(animation_points)
            and animation_points[next_index][0] < current_time
        ):
            next_index += 1
        if next_index >= len(animation_points):
            next_index = len(animation_points) - 1
        last_point = animation_points[next_index - 1]
        next_point = animation_points[next_index]

        t = (current_time - last_point[0]) / (next_point[0] - last_point[0])
        t = 1 if t > 1 else t
        t = 0 if t < 0 else t
        theta_1 = (next_point[1] - last_point[1]) * t + last_point[1]
        theta_2 = (next_point[2] - last_point[2]) * t + last_point[2]

        x = [
            config["origin"][0],
            config["origin"][0] + config["shoulder"]["length"] * math.cos(theta_1),
            config["origin"][0]
            + config["shoulder"]["length"] * math.cos(theta_1)
            + config["elbow"]["length"] * math.cos(theta_1 + theta_2),
        ]
        y = [
            config["origin"][1],
            config["origin"][1] + config["shoulder"]["length"] * math.sin(theta_1),
            config["origin"][1]
            + config["shoulder"]["length"] * math.sin(theta_1)
            + config["elbow"]["length"] * math.sin(theta_1 + theta_2),
        ]
        ax.clear()
        ax.plot([-2, 2], [0, 0])
        ax.plot(x, y)

        ax.set_xlim([-2, 2])
        ax.set_ylim([-0.5, 2.6])

        height_limit = config["constraints"]["height_limit"]["args"][0]
        max_neg_x = config["constraints"]["max_extension_neg"]["args"][0]
        max_pos_x = config["constraints"]["max_extension_pos"]["args"][0]
        ax.plot([max_neg_x, max_neg_x], [-2, 2])
        ax.plot([max_pos_x, max_pos_x], [-2, 2])
        ax.plot([-2, 2], [height_limit, height_limit])

        arm_width = abs(config["constraints"]["armSupport"]["args"][0] - config["constraints"]["armSupport"]["args"][2])
        arm_height = abs(config["constraints"]["armSupport"]["args"][1] - config["constraints"]["armSupport"]["args"][3])
        arm_rect = patches.Rectangle((config["constraints"]["armSupport"]["args"][0], config["constraints"]["armSupport"]["args"][1]), arm_width, arm_height)
        ax.add_patch(arm_rect)

        robot_width = abs(config["constraints"]["robotBody"]["args"][0] - config["constraints"]["robotBody"]["args"][2])
        robot_height = abs(config["constraints"]["robotBody"]["args"][1] - config["constraints"]["robotBody"]["args"][3])
        robot_rect = patches.Rectangle((config["constraints"]["robotBody"]["args"][0], config["constraints"]["robotBody"]["args"][1]), robot_width, robot_height)
        ax.add_patch(robot_rect)


    # Show plot
    animation = matplotlib.animation.FuncAnimation(fig, animate, interval=10)
    plt.show()