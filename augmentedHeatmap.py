from mapGenerator import SimulationMap
from _searcherTrajectories import a_star_search, generateSearcherMap
from heatmapGenerator import LostPeopleHeatmap
import numpy as np
import matplotlib.pyplot as plt


def augmentedHeatmap(searcherMap, heatmap, tuner):

    mu = 2*np.average(heatmap)

    #numerator = heatmap[:, :] + mu
    #denominator = 1 + tuner*searcherMap

    augMap = (heatmap[:, :] + mu)/(1 + tuner*searcherMap)

    return augMap

def humanSearcherModel(terrainMap:SimulationMap, waypoints_List, init_limit=0.2, lim_inc=0.01):
    paths = []
    searcherMap = np.zeros(terrainMap.terrainHeight.shape).astype(np.int8)
    for startPos, targets in waypoints_List:
        path = a_star_search(targets, startPos[0], startPos[1], init_limit, lim_inc)
        paths.append(path)
        searcherMap = searcherMap + generateSearcherMap(terrainMap, path)

    searcherMap = np.minimum(searcherMap, 1)

    return searcherMap, paths
    

def main():
    map = SimulationMap(400, 10)
    map.loadMap()
    lost_people_init_loc = [(20, 20), (12, 8), (10, 27), (28, 10), (26, 20), ]

    searcher_start_pos = [(7, 39), (20, 39), (33, 39)]
    searcher_targets=[[(0, 26), (13, 13), (7, 0)], \
               [(13, 26), (26, 13), (20, 0)], \
                [(26, 26), (39, 13), (33, 0)]]
    waypoints = zip(searcher_start_pos, searcher_targets)

    heatmap = LostPeopleHeatmap(map, lost_people_init_loc, iterations=25000, blur='yes')

    searcherMap, paths = humanSearcherModel(map, waypoints)

    augHeatmap = augmentedHeatmap(searcherMap, heatmap, 99)

    plt.rcParams["figure.figsize"] = [14.00, 7.00]
    plt.rcParams["figure.autolayout"] = True

    # Blur is to spread out possible areas of detection so that Drone paths can detect
    # hotspots easier
    plt.subplot(2, 2, 1)
    plt.imshow(map.terrainHeight, cmap="terrain_r")
    plt.title("Terrain Map")
    plt.colorbar()

    plt.subplot(2, 2, 2)
    plt.imshow(heatmap, cmap='hot')
    plt.title("Original Heatmap")
    plt.colorbar()
    # plt.colorbar()

    plt.subplot(2, 2, 3)
    plt.imshow(searcherMap, cmap='gray')
    plt.title("Searcher Paths")
    plt.colorbar()

    # Blur is to spread out possible areas of detection so that Drone paths can detect
    # hotspots easier
    plt.subplot(2, 2, 4)
    plt.imshow(augHeatmap, cmap="hot")
    plt.title("Augmented Heatmap")
    plt.colorbar()

    plt.show()
    
    pass



if __name__ == '__main__':
    main()