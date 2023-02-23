# Python File to generate Ground Searcher Trajectories based on Terrain Information

from mapGenerator import SimulationMap
import math
import numpy as np
import matplotlib.pyplot as plt

# STATIC MAP TO BE USED IS PROVIDED IN THE FILE "TestMap.npy"
map = SimulationMap(400, (10, 10))
map.loadMap()

# start position
_start_x = 0
_start_y = 10
# sample target lists
target_list_a = [(15, 5), (17, 18), (18, 25)]
target_list_b = [(10, 10), (10, 27), (16, 22), (27, 19), (37, 8)]
_targets = target_list_a


# find which of the closest adjacent nodes is closest to the current target
def f_waypoint(x, target):
    adj = map.adjacentNodes(x, 400)
    num_adj = len(adj)
    distance = list((math.dist(adj[i], target)) for i in range(num_adj))
    update = np.argmin(distance)
    return adj[update]


# take the terrain into account when traveling
def f_gradient(x, prev_x, target, max_dx):
    adj = map.adjacentNodes(x, 400)
    num_adj = len(adj)
    current_height = map.terrainHeight[x]
    height_diff = list(abs(current_height - map.terrainHeight[adj[i]]) for i in range(num_adj))
    distance = list((math.dist(adj[i], target)) for i in range(num_adj))
    for i in range(num_adj):
        # remove all inaccessible adjacent nodes from consideration
        if height_diff[i] > max_dx:
            distance[i] = 9999
    # return closest accessible node
    update = np.argmin(distance)
    return adj[update]


def generate_paths(targets, start_x, start_y):
    # init
    start_position = start_x, start_y
    num_targets = len(targets)
    path = [start_position]
    x_path = [start_position[0]]
    y_path = [start_position[1]]
    position = old_p = start_position
    init_limit = 1.0  # max gradient we want our rescuers to be crossing
    limit_inc = 0.05  # how fast we want the gradient limit to increase
    max_count = 20  # retry attempts before increasing gradient limit

    # run operation until we are out of targets
    for i in range(num_targets):
        counter = 0
        current_distance = math.dist(position, targets[i])
        count_threshold = current_distance + max_count
        limit = init_limit
        # move until we reach target location
        while position != targets[i]:
            older_p = old_p
            old_p = position
            position = f_waypoint(position, targets[i])
            # if proposed location is inaccessible according to current limits
            # switch to gradient pathing instead
            if abs(map.terrainHeight[old_p] - map.terrainHeight[position]) > limit:
                position = f_gradient(old_p, older_p, targets[i], limit)
            counter = counter + 1
            # to prevent an infinite loop, gradually increase the gradient threshold
            if counter > count_threshold:
                counter = current_distance
                limit = limit + limit_inc
                print("limit increased to ", limit)
            # record current location so we can retrace our steps
            x_path.append(position[0])
            y_path.append(position[1])
            path.append(position)
        print("reached ", targets[i])

    # plot the results
    # plot the path the searcher took
    plt.plot(x_path, y_path)
    # mark the target locations on the plot
    x_targets = []
    y_targets = []
    for i in range(num_targets):
        temp = targets[i]
        x_targets.append(temp[0])
        y_targets.append(temp[1])
    plt.plot(x_targets, y_targets, 'ro')
    # plot the starting location
    plt.plot(start_x, start_y, 'yd')


def run():
    plt.imshow(map.terrainHeight, cmap="gray")
    generate_paths(_targets, _start_x, _start_y)
    plt.show()


run()
