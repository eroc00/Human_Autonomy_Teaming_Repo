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
target_list_c = [(12, 10), (13, 19)]
_targets = target_list_c


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


# generate paths based on gradient and waypoints
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
    plot_paths(x_path, y_path, targets, start_x, start_y)


def plot_paths(path_x, path_y, targets_list, start_x, start_y):
    # plot the path the searcher took
    plt.plot(path_x, path_y)
    # mark the target locations on the plot
    x_targets = []
    y_targets = []
    for i in range(len(targets_list)):
        temp = targets_list[i]
        x_targets.append(temp[0])
        y_targets.append(temp[1])
    plt.plot(x_targets, y_targets, 'ro')
    # plot the starting location
    plt.plot(start_x, start_y, 'yd')


##################################################################################
# below functions/classes are for the A* Search Function #########################
##################################################################################
class Node:
    # class for A* search
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0


# Inputs:
#  targets - list of waypoints
#  start_x, start_y - starting x, y coordinates
#  init_limit - initial max gradient the searcher can traverse
#  tolerance - amount of times we can get stuck in one place before action is taken
#  tol_inc - how quickly we want to increase the gradient limit the searcher can traverse
def a_star_search(targets, start_x, start_y, init_limit, tolerance, tol_inc):
    start_position = start_x, start_y
    limit = init_limit
    path = []
    x_path = []
    y_path = []
    num_targets = len(targets)
    for i in range(num_targets):
        stuck_count = 0
        # check if we have just started, use start position as start_node if so
        if i == 0:
            start_node = Node(None, start_position)
        # set start node to last target node if not just starting
        else:
            start_node = Node(None, targets[i-1])
        end_node = Node(None, targets[i])
        print("start position: ", start_node.position)
        print("current target: ", end_node.position)
        open_list = []
        closed_list = []
        open_list.append(start_node)

        while len(open_list) > 0:
            # find current node, and move it from open to closed list
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index
            # print(current_node.position)
            open_list.pop(current_index)
            closed_list.append(current_node)

            # check if reached current target
            if current_node.position == end_node.position:
                print("reached ", end_node.position)
                path_section = []
                current = current_node
                # backtrack to get the path traveled from previous waypoint
                while current is not None:
                    path_section.append(current.position)
                    current = current.parent
                section_length = len(path_section)
                # update the path by adding this segment to it
                for j in range(section_length):
                    path.append(path_section[section_length - j - 1])
                break

            # check if searcher is stuck, and relax restrictions if so
            if current_index != 0:
                previous_node = current_node.parent
                if current_node.position == previous_node.position:
                    stuck_count = stuck_count + 1
                    if stuck_count > tolerance:
                        limit = limit + tol_inc
                        stuck_count = 0

            # generate child nodes
            children = []
            for adjacent_node in map.adjacentNodes(current_node.position, 400):
                # check if child node is accessible from current node
                if abs(map.terrainHeight[current_node.position] - map.terrainHeight[adjacent_node]) < limit:
                    # check if child node is in bounds
                    if adjacent_node[0] <= 20 or adjacent_node[0] >= 0 or adjacent_node[1] <= 20 or adjacent_node[1] >= 0:
                        new_node = Node(current_node, adjacent_node)
                        children.append(new_node)
                        # print("new child ", new_node.position)

            # if child is not in either open_list or closed _list generate f g h and add it to open_list
            # if child is already in open_list check if it has a lower g value
            # if so change its parent to current node and recalculate its f g and h
            for child in children:
                if child not in closed_list:
                    child.g = current_node.g + 1
                    child.h = math.dist(child.position, end_node.position)
                    child.f = child.g + child.h
                    flag = 0
                    for open_node in open_list:
                        if child.position == open_node.position and child.g > open_node.g:
                            flag = 1
                    if flag == 0:
                        open_list.append(child)
                        # print("add ", child.position, " to open list")

    # plot results
    print(path)
    path_length = len(path)
    for i in range(path_length):
        temp = path[i]
        x_path.append(temp[0])
        y_path.append(temp[1])
    plot_paths(x_path, y_path, targets, start_x, start_y)


def run():
    plt.imshow(map.terrainHeight, cmap="gray")

    # orginial algorithm based on waypoint and gradient following method
    # generate_paths(_targets, _start_x, _start_y)

    # a star inspired search method
    a_star_search(_targets, _start_x, _start_y, 1, 5, 0.1)

    plt.show()


run()
