from mapGenerator import SimulationMap
import searcherTrajectories as ST
#from searcherTrajectories import a_star_uav as AStarUAV, a_star_search as GroundSearcher
import heatmapGenerator as HMG
#import UAVTrajectories as RRTPP
import augmentedHeatmap as AHM
from BellmanFordOptimizer import BellmanFord as PathOptimizer
import matplotlib.pyplot as plt
import time, math, numpy as np, pandas

plotBool = True
recordTimeExecution = not plotBool

class Default:
    def __init__(self, params={}):
        self.params = params
        self.set_defaults()
        pass

    def set_defaults(self):
        # Map Parameters
        self.params.setdefault( 'area', 400 )
        self.params.setdefault( 'res', 10 )
        self.params.setdefault( 'zlims', (0, 3) ) # Minimum and Maximum height of terrain
        self.params.setdefault( 'numObstacles', 0 )
        self.params.setdefault( 'numDrones', 3)

        # Searcher Trajectories Parameters

        #self.params.setdefault( 'searchersInit', [(7, 39), (20, 39), (33, 39)] ) # Initial starting points of searchers
        #self.params.setdefault( 'searchersWaypoints', [[(0, 26), (13, 13), (7, 0)], \
        #  [(13, 26), (26, 13), (20, 0)], \
        # [(26, 26), (39, 13), (33, 0)]] )

        self.params.setdefault( 'init_limit', 0.2)
        self.params.setdefault( 'lim_inc', 0.1)

        # Heatmap Generation Parameters
        self.params.setdefault( 'iter', 25000 )
        self.params.setdefault( 'alpha', 9)

        # Optimization Parameters
        self.params.setdefault( 'DroneWaypoints', 2)
        self.params.setdefault( 'aH', 0.5)
        self.params.setdefault( 'aL', 0.5)

class PathPlanner(SimulationMap):
    heatmap = None
    searcherMap = None
    pathList = []
    pathCost = []
    params = {}

    def __init__(self, load='yes', filename="TestMap.npy", params=Default().params, plot=True):
                 #size=400, resolution=(10, 10), minHeight=0, maxHeight=10, numObstacles=6) -> None:
        self.params = params
        if load != 'yes':
            super().__init__(size=self.params['area'],\
                              resolution=self.params['res'],\
                                minHeight=self.params['zlims'][0],\
                                      maxHeight=self.params['zlims'][1],\
                                          numObstacles=self.params['numObstacles'])

        else:
            self.loadMap(filename)

        #p = Default()
        
        self.optimizer = PathOptimizer()
        self.plotBool = plot
        if self.plotBool:
            fig, self.ax = plt.subplots()
            self.ax.set_aspect('equal')
            pass

    def augmentHeatmap(self):
        try:
            self.heatmap = AHM.augmentedHeatmap(self.searcherMap, self.heatmap, self.params['alpha'])
        except TypeError:
            print("Heatmap is not initialized. Create a heatmap by call generateHeatmap() first.")

    def generateHeatmap(self, lostPersonsLocations):
        self.heatmap = HMG.LostPeopleHeatmap(self, lostPersonsLocations, self.params['iter'], blur='yes')


    def executeSearcherPaths(self, wps):
        # a star inspired search method
        #start_pos = []
        #target_points = []
        self.pathList = []
        self.pathCost = []
        self.waypointsList = wps.copy()
        for waypoint in wps:
            start = waypoint.pop(0)
            path = ST.a_star_search(self, waypoint, start[0], start[1], self.params['init_limit'], self.params['lim_inc'])
            self.pathList.append(path[0])
            self.pathCost.append(path[1])

        points = sum(self.pathList, [])
        self.searcherMap = ST.generateSearcherMap(self, points)
    
    def showHeatmap(self):
        plt.imshow(self.heatmap, cmap='hot')
        plt.title("Heatmap")
        plt.colorbar()
        plt.show()

    def showSearcherPaths(self):
         # plot the path the searcher took
        plt.imshow(self.heatmap, cmap='hot')
        plt.title("Searcher Trajectories")
        plt.colorbar()
        for waypoints, path in zip(self.waypointsList, self.pathList):     
            
            # Plot path       
            x_path = [point[0] for point in path]
            y_path = [point[1] for point in path]
            plt.plot(x_path, y_path, color="#406142")

            # Mark Waypoints on map
            x_targets = [point[0] for point in waypoints]
            y_targets = [point[1] for point in waypoints]
            #plt.scatter(x_targets, y_targets)

            # plot the starting location
            start_x, start_y = path[0]
            plt.plot(start_x, start_y, 'yd')
        
        plt.show()

    def showSearcherBinaryMap(self):
        plt.imshow(self.searcherMap, cmap='gray')
        plt.title("Searcher Binary Map")
        plt.colorbar()
        plt.show()

    def generateDronePaths(self, startendPoints):
        paths = []

        gridlen = int(self.maplen/self.res) - 1
        segmentLen = int(gridlen/len(startendPoints))

        i = 0
        for start, end in startendPoints:

            domain = (i*segmentLen, (i+1)*segmentLen)

            self.optimizer.initGraph(self, start, end, self.params["DroneWaypoints"], domain, self.params['aH'], self.params['aL'])
            self.optimizer.optimize()
            paths.append(self.optimizer.optimalPath())

            if self.plotBool:
                x_coords = [point[0] for point in self.optimizer.waypoints]
                y_coords = [point[1] for point in self.optimizer.waypoints]

                self.ax.scatter(x_coords, y_coords, c='g')

            i += 1

        return paths
    
    def plotPaths(self, points, color=None):
        # plot the path the searcher took
        start = points[0]
        path_x = [point[0] for point in points]
        path_y = [point[1] for point in points]

        if color is None:
            self.ax.plot(path_x, path_y)
        else:
            self.ax.plot(path_x, path_y, color=color)

        # plot the starting location
        #self.ax.plot(start[0], start[1], 'yd')

def pathPlannerVisualization():
    # Last seen locations of lost people based on a 40x40 map
    lost_people_init_loc = [(20, 20), (12, 8), (10, 27), (28, 10), (15, 20)]
    #lost_people_init_loc = [(3, 14), (10, 10), (13, 4)]

    

    """
    numSearchers = 3
    for i in range(1, numSearchers+1):
        searcherWaypoints = []
        for p in range(4):
            searcherWaypoints.append( ( , p*(map.size//map.res)) )
    """

    searcher1_waypoints = [(7, 39), (10, 26), (1, 13), (7, 0)]
    searcher2_waypoints = [(20, 39), (25, 26), (13, 13), (20, 0)]
    searcher3_waypoints = [(33, 39), (26, 26), (39, 13), (33, 0)]

    waypoints = [searcher1_waypoints,\
                 searcher2_waypoints,\
                 searcher3_waypoints]

    
    customSettings = Default()
    #customSettings.params['DroneWaypoints'] = 4
    #customSettings.params['numObstacles'] = 2
    customSettings.params['numDrones'] = 3
    planner = PathPlanner(load="yes", filename="TestMap", params=customSettings.params)

    # Initialize Drone start and end points
    dronePointsInit = []
    mapWidth = (customSettings.params['area']//customSettings.params['res'])-1
    segmentWidth = mapWidth//customSettings.params['numDrones']
    for i in range(customSettings.params['numDrones']):
        droneX = segmentWidth//2 + i*segmentWidth
        startPoint = (droneX, mapWidth)
        endPoint = (droneX, 0)
        dronePointsInit.append((startPoint, endPoint))

    if recordTimeExecution:
        print(f"\nTIMING THE EXECUTION OF PATH PLANNER NOW...\n")
    st = time.time()

    #print("GENERATING INITIAL HEATMAP...")
    planner.generateHeatmap(lost_people_init_loc)
    if plotBool:
        planner.showHeatmap()
        pass
    
    #print("EXECUTING SEARCHER PATHS BASED ON WAYPOINTS...")
    planner.executeSearcherPaths(waypoints)
    if plotBool:
        #planner.showSearcherPaths()
        #planner.showSearcherBinaryMap()
        pass

    #print("AUGMENTING HEATMAP...")
    planner.augmentHeatmap()
    planner.showSearcherPaths()
    if plotBool:
        #planner.showHeatmap()
        pass

    #print("GENERATING DRONE PATHS...")
    if plotBool:
        fig, planner.ax = plt.subplots()
    optimalDronePaths = planner.generateDronePaths(dronePointsInit)

    # Plot Final Map
    if plotBool:
        print("PLOTTING...")

        plt.imshow(planner.heatmap, cmap='hot')
        plt.title("Optimized Drone Paths")
        plt.colorbar()

        # Plot Anticipated Human Ground Searcher paths
        for humanPath in planner.pathList:
            planner.plotPaths(humanPath, color='#406142')

        # Plot Optimized Drone Paths
        for path, c in zip(optimalDronePaths, ['#1987E6', '#1921E6', '#19E6DE']):
            planner.plotPaths(path, color=c)
        #planner.ax.imshow(planner.heatmap, cmap="hot")
        num_obstacles = len(planner.obstacles)

        for obstacle in planner.obstacles:
            coords, radius = ST.scale_obstacle(obstacle)
            circle = plt.Circle(coords, radius, color='y')
            planner.ax.add_patch(circle)
        
        plt.show()
        print("DONE.")

    duration = time.time() - st

    # Measure performance...
    heatAcc = 0
    length = 0
    for path in optimalDronePaths:
        # Heatmap Accumulation
        startPoint = path.pop(0)
        heatAcc += planner.heatmap[startPoint]
        for point in path:
            heatAcc += planner.heatmap[point]
            planner.heatmap[point] = 0 # Do not count spots that have already been visited by other drones
            length += math.dist(startPoint, point)

    print(f"Drone Count = {customSettings.params['numDrones']}")
    print(f"Total Heatmap accumulation = {heatAcc:.3f}")
    print(f"Total Length Covered = {(length * planner.res):.2f} meters")
    print(f"Execution Time = {duration:.3f}")

def pathPlannerStats(numDrones):
    # Last seen locations of lost people based on a 40x40 map
    lost_people_init_loc = [(20, 20), (12, 8), (10, 27), (28, 10), (15, 20)]

    searcher1_waypoints = [(7, 39), (10, 26), (1, 13), (7, 0)]
    searcher2_waypoints = [(20, 39), (25, 26), (13, 13), (20, 0)]
    searcher3_waypoints = [(33, 39), (26, 26), (39, 13), (33, 0)]

    waypoints = [searcher1_waypoints,\
                 searcher2_waypoints,\
                 searcher3_waypoints]

    
    customSettings = Default()
    customSettings.params['numDrones'] = numDrones
    planner = PathPlanner(load="yes", filename="TestMap", params=customSettings.params, plot=False)

    # Initialize Drone start and end points
    dronePointsInit = []
    mapWidth = (customSettings.params['area']//customSettings.params['res'])-1
    segmentWidth = mapWidth//customSettings.params['numDrones']
    for i in range(customSettings.params['numDrones']):
        droneX = segmentWidth//2 + i*segmentWidth
        startPoint = (droneX, mapWidth)
        endPoint = (droneX, 0)
        dronePointsInit.append((startPoint, endPoint))

    st = time.time()

    #print("GENERATING INITIAL HEATMAP...")
    planner.generateHeatmap(lost_people_init_loc)
    
    #print("EXECUTING SEARCHER PATHS BASED ON WAYPOINTS...")
    planner.executeSearcherPaths(waypoints)

    #print("AUGMENTING HEATMAP...")
    planner.augmentHeatmap()

    #print("GENERATING DRONE PATHS...")
    #optimalDronePaths = planner.generateDronePaths(dronePointsInit)

    # Manually generated waypoints for drones
    drone1Waypoints = [(7, 39), (7, 13), (9, 26), (7, 0)]
    drone2Waypoints = [(20, 39), (21, 13), (17, 26), (20, 0)]
    drone3Waypoints = [(33, 39), (25, 13), (32, 26), (33, 0)]

    droneWaypoints = [drone1Waypoints, drone2Waypoints, drone3Waypoints]
    optimalDronePaths = []
    for wp in droneWaypoints:
        start = wp.pop(0)
        optimalDronePaths.append(ST.a_star_uav(planner, planner.heatmap, wp, start[0], start[1]))

    duration = time.time() - st

    # Measure performance...
    heatAcc = 0
    length = 0
    for path in optimalDronePaths:
        # Heatmap Accumulation
        startPoint = path.pop(0)
        heatAcc += planner.heatmap[startPoint]
        for point in path:
            heatAcc += planner.heatmap[point]
            planner.heatmap[point] = 0 # Do not count spots that have already been visited by other drones
            length += math.dist(startPoint, point)

    #      Heat Accumulation, Distance traveled by drones, execution time
    return heatAcc, (length*planner.res), duration

    """print(f"Drone Count = {customSettings.params['numDrones']}")
    print(f"Total Heatmap accumulation = {heatAcc:.3f}")
    print(f"Total Length Covered = {(length * planner.res):.2f} meters")
    print(f"Execution Time = {duration:.3f}")"""


if __name__ == "__main__":
    pathPlannerVisualization()
    ppMetrics = []

    """
    iterations = 1
    for numDrones in range(1, 7):
        print(f"Running Algorithm with {numDrones} drones...")
        for i in range(iterations):
            heatAcc, dist, duration = pathPlannerStats(numDrones)
            ppMetrics.append([numDrones, heatAcc, dist, duration])

    #ppMetrics = np.array(ppMetrics)
    resultsdf = pandas.DataFrame(ppMetrics, columns=["Drone Count", "Total Heatmap Accumulation", "Total Distance Traveled", "Code Execution Time"])
    resultsdf.to_excel("Algorithm Performance.xlsx")
    """
            


    # One Problem with this approach is that it may not scale well for larger grid sizes

    
    





