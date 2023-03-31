# Python File to generate Ground Searcher Trajectories based on Terrain Information

from mapGenerator import SimulationMap
from heatmapGenerator import LostPeopleHeatmap
import math
import numpy as np
import matplotlib.pyplot as plt


# find which of the closest adjacent nodes is closest to the current target
def f_waypoint(x, target):
    adj = map.adjacentNodes(x)
    num_adj = len(adj)
    distance = list((math.dist(adj[i], target)) for i in range(num_adj))
    update = np.argmin(distance)
    return adj[update]


# take the terrain into account when traveling
def f_gradient(x, prev_x, target, max_dx):
    adj = map.adjacentNodes(x)
    num_adj = len(adj)
    current_height = map.terrainHeight[x]
    height_diff = list(abs(current_height - map.terrainHeight[swap(adj[i])]) for i in range(num_adj))
    distance = list((math.dist(adj[i], target)) for i in range(num_adj))
    for i in range(num_adj):
        # remove all inaccessible adjacent nodes from consideration
        if height_diff[i] > max_dx:
            distance[i] = 9999
    # return closest accessible node
    update = np.argmin(distance)
    return adj[update]


# generate paths based on gradient and waypoints
def generate_paths(targets, start_x, start_y, init_limit, limit_inc):
    # init
    start_position = start_x, start_y
    num_targets = len(targets)
    path = [start_position]
    x_path = [start_position[0]]
    y_path = [start_position[1]]
    position = old_p = start_position
    # init_limit = 1.0  # max gradient we want our rescuers to be crossing
    # limit_inc = 0.05  # how fast we want the gradient limit to increase
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
            if abs(map.terrainHeight[swap(old_p)] - map.terrainHeight[swap(position)]) > limit:
                position = f_gradient(old_p, older_p, targets[i], limit)
            counter = counter + 1
            # to prevent an infinite loop, gradually increase the gradient threshold
            if counter > count_threshold:
                counter = current_distance
                limit = limit + limit_inc
                # print("limit increased to ", limit)
            # record current location so we can retrace our steps
            x_path.append(position[0])
            y_path.append(position[1])
            path.append(position)
        print("reached ", targets[i])

    op_path, difficulty = path_details(path)

    # plot the results
    plot_paths(x_path, y_path, targets, start_x, start_y, 'b')

    return op_path, difficulty


def path_details(path):
    # path details
    # print("path taken: ", path)
    op_path = []
    for i in range(len(path)):
        if path[i] not in op_path:
            op_path.append(path[i])
        else:
            index = op_path.index(path[i])
            del op_path[index + 1:]
    print("path taken: ", op_path)
    # print("path length: ", len(op_path))
    gradient_cost = 0
    current_elevation = map.terrainHeight[swap(op_path[0])]
    for i in range(len(op_path)):
        prev_elevation = current_elevation
        current_elevation = map.terrainHeight[swap(op_path[i])]
        gradient_cost = gradient_cost + abs(current_elevation - prev_elevation)
    # print("path difficulty (vertical smoothness): ", gradient_cost)
    return op_path, gradient_cost


def plot_paths(path_x, path_y, targets_list, start_x, start_y, color):
    # plot the path the searcher took
    plt.plot(path_x, path_y, color)
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


def swap(x):
    y = (x[1], x[0])
    return y


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
def a_star_search(targets, start_x, start_y, init_limit, lim_inc):
    start_position = start_x, start_y
    path = []
    x_path = []
    y_path = []
    num_targets = len(targets)
    for i in range(num_targets):
        limit = init_limit
        # check if we have just started, use start position as start_node if so
        if i == 0:
            start_node = Node(None, start_position)
        # set start node to last target node if not just starting
        else:
            start_node = Node(None, targets[i - 1])
        end_node = Node(None, targets[i])
        # print("start position: ", start_node.position)
        # print("current target: ", end_node.position)
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
            # print("current node ", current_node.position)
            # print(len(open_list))
            open_list.pop(current_index)
            closed_list.append(current_node)

            # check if reached current target
            if current_node.position == end_node.position:
                # print("reached ", end_node.position)
                reached_target = 1
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

            # generate child nodes
            children = []
            for adjacent_node in map.adjacentNodes(current_node.position):
                # check if child node is accessible from current node
                if abs(map.terrainHeight[swap(current_node.position)] - map.terrainHeight[swap(adjacent_node)]) < limit:
                    # check if child node is in bounds
                    if adjacent_node[0] >= 0 and adjacent_node[1] >= 0:
                        new_node = Node(current_node, adjacent_node)
                        children.append(new_node)

            # if child is not in either open_list or closed _list generate f g h and add it to open_list
            # if child is already in open_list check if it has a lower g value
            # if so change its parent to current node and recalculate its f g and h
            for child in children:
                flag = 0
                for closed_node in closed_list:
                    if child.position == closed_node.position:
                        flag = 1
                if flag == 0:
                    if child.position[0] == current_node.position[0] or child.position[1] == current_node.position[1]:
                        child.g = current_node.g + 1
                    else:
                        child.g = current_node.g + math.sqrt(2)
                    child.h = math.dist(child.position, end_node.position)
                    child.f = child.g + child.h
                    for open_index, open_node in enumerate(open_list):
                        if child.position == open_node.position:
                            flag = 1
                            if child.g < open_node.g:
                                open_list[open_index] = child
                    if flag == 0:
                        open_list.append(child)
                        # print("add ", child.position, " to open list")
            # if target can not be reached with current settings, relax restrictions
            if len(open_list) == 0:
                # print("target unreachable with current settings of ", limit)
                limit = limit + lim_inc
                open_list = []
                closed_list = []
                open_list.append(start_node)

    path_length, difficulty = a_star_stats(path)

    # plot results
    for i in range(path_length):
        temp = path[i]
        x_path.append(temp[0])
        y_path.append(temp[1])
    plot_paths(x_path, y_path, targets, start_x, start_y, 'g')

    return path, difficulty


# a star stats
def a_star_stats(path):
    path_length = len(path)
    gradient_cost = 0
    current_elevation = map.terrainHeight[swap(path[0])]
    for i in range(path_length):
        prev_elevation = current_elevation
        current_elevation = map.terrainHeight[swap(path[i])]
        gradient_cost = gradient_cost + abs(current_elevation - prev_elevation)
    print("path taken ", path)
    # print("path length: ", path_length)
    # print("path difficulty (vertical smoothness): ", gradient_cost)
    return path_length, gradient_cost


# UAV search with heat map and obstacles
def a_star_uav(targets_in, start_x, start_y):
    start_position = start_x, start_y
    no_path = 0
    path = []
    x_path = []
    y_path = []
    targets = []
    num_targets = len(targets_in)
    # check if any targets are in an obstacle and discard them
    for i in range(num_targets):
        flag = 0
        for obstacle in map.obstacles:
            ob_cords, ob_radius = scale_obstacle(obstacle)
            if math.dist(targets_in[i], ob_cords) <= ob_radius:
                flag = 1
                print("remove ", targets_in[i])
        if flag != 1:
            targets.append(targets_in[i])
    num_targets = len(targets)
    for i in range(num_targets):
        # check if we have just started, use start position as start_node if so
        if i == 0:
            start_node = Node(None, start_position)
        # set start node to last target node if not just starting
        else:
            start_node = Node(None, targets[i - 1])
        end_node = Node(None, targets[i])
        # print("start position: ", start_node.position)
        # print("current target: ", end_node.position)
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
            # print("current node ", current_node.position)
            # print(len(open_list))
            open_list.pop(current_index)
            closed_list.append(current_node)

            # check if reached current target
            if current_node.position == end_node.position:
                # print("reached ", end_node.position)
                reached_target = 1
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

            # generate child nodes
            children = []
            for adjacent_node in map.adjacentNodes(current_node.position):
                # check if in obstacle
                child_flag = 0
                for obstacle in map.obstacles:
                    ob_cords, ob_radius = scale_obstacle(obstacle)
                    if math.dist(adjacent_node, ob_cords) <= ob_radius:
                        child_flag = 1
                # check if child node is in bounds
                if adjacent_node[0] >= 0 and adjacent_node[1] >= 0:
                    new_node = Node(current_node, adjacent_node)
                    if child_flag != 1:
                        children.append(new_node)

            # if child is not in either open_list or closed _list generate f g h and add it to open_list
            # if child is already in open_list check if it has a lower g value
            # if so change its parent to current node and recalculate its f g and h
            for child in children:
                flag = 0
                for closed_node in closed_list:
                    if child.position == closed_node.position:
                        flag = 1
                if flag == 0:
                    if child.position[0] == current_node.position[0] or child.position[1] == current_node.position[1]:
                        child.g = current_node.g + 1
                        # incentive for pathing through hot areas
                        child_value = min(2000, heatmap[swap(child.position)]) - 5
                        child_value = child_value / 1000
                        if child_value != 0:
                            print(child_value, child.position)
                        new_g = child.g - child_value
                        child.g = max(new_g, 0)  # don't want negative g score
                    else:
                        child.g = current_node.g + math.sqrt(2)
                    child.h = math.dist(child.position, end_node.position)
                    child.f = child.g + child.h
                    for open_index, open_node in enumerate(open_list):
                        if child.position == open_node.position:
                            flag = 1
                            if child.g < open_node.g:
                                open_list[open_index] = child
                    if flag == 0:
                        open_list.append(child)
                        # print("add ", child.position, " to open list")
            # if target can not be reached with current settings, relax restrictions
            if len(open_list) == 0:
                print("target ", targets[i], " is unreachable")
                no_path = 55
                break
        if no_path == 55:
            break

    # plot results
    for i in range(len(path)):
        temp = path[i]
        x_path.append(temp[0])
        y_path.append(temp[1])
    plot_paths(x_path, y_path, targets, start_x, start_y, 'b')


def scale_obstacle(obstacle):
    in_cords = obstacle[0]
    x = in_cords[0] / 10
    y = in_cords[1] / 10
    radius = obstacle[1] / 10
    out_cords = (x, y)
    return out_cords, radius


def plot_obstacles():
    num_obstacles = len(map.obstacles)
    fig, ax = plt.subplots()
    plt.xlim(0, 40)
    plt.ylim(0, 40)
    for i in range(num_obstacles):
        obstacle = map.obstacles[i]
        cords, radius = scale_obstacle(obstacle)
        # print(cords, radius)
        circle = plt.Circle(cords, radius, color='y')
        ax.add_patch(circle)


def init_heat_map(_map, init_locations):
    heatmap = LostPeopleHeatmap(_map, init_locations, blur='yes')
    return heatmap


############################################################################
# execute ##################################################################
# STATIC MAP TO BE USED IS PROVIDED IN THE FILE "TestMap.npy"
map = SimulationMap(400, (10, 10))
map.loadMap()

# start positions
_start_x = 0
_start_y = 10
_start_x2 = 0
_start_y2 = 30
# sample target lists
target_list_a = [(15, 5), (17, 18), (18, 25)]
target_list_b = [(10, 10), (10, 27), (16, 22), (27, 19), (37, 8)]
target_list_c = [(12, 10), (13, 19), (18, 6)]
target_list_cross_map_top = [(10, 10), (20, 10), (30, 10), (39, 10)]
target_list_cross_map_bottom = [(10, 30), (20, 30), (30, 30), (39, 30)]
_targets = target_list_cross_map_top
_targets2 = target_list_cross_map_bottom
_targets3 = [(10, 14), (15, 15), (20, 20), (25, 25), (30, 30)]
# heat map init locations
lost_people_init_loc = [(20, 20), (12, 8), (10, 27), (28, 10), (26, 20)]
heatmap = init_heat_map(map, lost_people_init_loc)


def run_1():
    # original algorithm based on waypoint and gradient following method
    print("waypoint/gradient search")
    # generate_paths(_targets, _start_x, _start_y, 1, 0.05)
    path_1, diff_1 = generate_paths(_targets, _start_x, _start_y, 0.2, 0.1)
    path_2, diff_2 = generate_paths(_targets2, _start_x2, _start_y2, 0.2, 0.1)
    print("length: ", len(path_1) + len(path_2))
    print("difficulty: ", diff_1 + diff_2)


def run_2():
    # a star inspired search method
    print("A* search")
    path_1, diff_1 = a_star_search(_targets, _start_x, _start_y, 0.1, 0.1)
    path_2, diff_2 = a_star_search(_targets2, _start_x2, _start_y2, 0.1, 0.1)
    print("length: ", len(path_1) + len(path_2))
    print("difficulty: ", diff_1 + diff_2)


def run_3():
    plot_obstacles()
    plt.imshow(heatmap, cmap="hot")
    a_star_uav(_targets3, 0, 0)


# plt.imshow(map.terrainHeight, cmap="gray")
plt.imshow(map.terrainHeight, cmap="terrain_r")
run_1()
run_2()
# run_3()
plt.show()
