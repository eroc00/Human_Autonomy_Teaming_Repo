from mapGenerator import SimulationMap
import searcherTrajectories as ST
#from searcherTrajectories import a_star_uav as AStarUAV, a_star_search as GroundSearcher
import heatmapGenerator as HMG
#import UAVTrajectories as RRTPP
import augmentedHeatmap as AHM
from BellmanFordOptimizer import BellmanFord as PathOptimizer
import matplotlib.pyplot as plt
import time

plotBool = False
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
        self.params.setdefault( 'numObstacles', 6 )

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

class PathPlanner(SimulationMap):
    heatmap = None
    searcherMap = None
    pathList = []
    pathCost = []
    params = {}

    def __init__(self, load='yes', filename="TestMap.npy"):
                 #size=400, resolution=(10, 10), minHeight=0, maxHeight=10, numObstacles=6) -> None:
        if load != 'yes':
            super().__init__(size=self.params['area'],\
                              resolution=self.params['res'],\
                                minHeight=self.params['zlims'][0],\
                                      maxHeight=self.params['zlims'][1],\
                                          numObstacles=self.params['numObstacles'])

        else:
            self.loadMap(filename)

        #p = Default()
        self.params = Default().params
        self.optimizer = PathOptimizer()
        if plotBool:
            fig, self.ax = plt.subplots()
            self.ax.set_aspect('equal')

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
        plt.imshow(planner.heatmap, cmap='hot')
        plt.title("Heatmap")
        plt.colorbar()
        plt.show()

    def showSearcherPaths(self):
         # plot the path the searcher took
        plt.imshow(planner.terrainHeight, cmap='terrain_r')
        plt.title("Searcher Trajectories")
        for waypoints, path in zip(self.waypointsList, self.pathList):     
            
            # Plot path       
            x_path = [point[0] for point in path]
            y_path = [point[1] for point in path]
            plt.plot(x_path, y_path)

            # Mark Waypoints on map
            x_targets = [point[0] for point in waypoints]
            y_targets = [point[1] for point in waypoints]
            plt.scatter(x_targets, y_targets)

            # plot the starting location
            start_x, start_y = path[0]
            plt.plot(start_x, start_y, 'yd')
        
        plt.show()

    def showSearcherBinaryMap(self):
        plt.imshow(planner.searcherMap, cmap='gray')
        plt.title("Searcher Binary Map")
        plt.colorbar()
        plt.show()

    def generateDronePaths(self, startendPoints, numTurnsPerDrone):
        paths = []

        gridlen = int(self.maplen/self.res) - 1
        segmentLen = int(gridlen/len(waypoints))

        i = 0
        for start, end in startendPoints:

            domain = (i*segmentLen, (i+1)*segmentLen)

            self.optimizer.initGraph(self, start, end, numTurnsPerDrone, domain)
            self.optimizer.optimize()
            paths.append(self.optimizer.optimalPath())

            if plotBool:
                x_coords = [point[0] for point in self.optimizer.waypoints]
                y_coords = [point[1] for point in self.optimizer.waypoints]

                self.ax.scatter(x_coords, y_coords, 200)

            i += 1

        return paths
    
    def plotPaths(self, points):
        # plot the path the searcher took
        start = points[0]
        path_x = [point[0] for point in points]
        path_y = [point[1] for point in points]

        self.ax.plot(path_x, path_y)

        # plot the starting location
        self.ax.plot(start[0], start[1], 'yd')




if __name__ == "__main__":

    # Last seen locations of lost people based on a 40x40 map
    lost_people_init_loc = [(20, 20), (12, 8), (10, 27), (28, 10), (15, 20)]

    searcher1_waypoints = [(7, 39), (10, 26), (1, 13), (7, 0)]
    searcher2_waypoints = [(20, 39), (25, 26), (13, 13), (20, 0)]
    searcher3_waypoints = [(33, 39), (26, 26), (39, 13), (33, 0)]

    waypoints = [searcher1_waypoints,\
                 searcher2_waypoints,\
                 searcher3_waypoints]

    dronePointsInit = [(wp[0], wp[len(wp)-1]) for wp in waypoints]

    """
    searcher_start_pos = [(7, 39), (20, 39), (33, 39)]
    searcher_targets=[[(0, 26), (13, 13), (7, 0)], \
               [(13, 26), (26, 13), (20, 0)], \
                [(26, 26), (39, 13), (33, 0)]]
    """
                
    planner = PathPlanner()

    if recordTimeExecution:
        print(f"\nTIMING THE EXECUTION OF PATH PLANNER NOW...\n")
    st = time.time()

    #print("GENERATING INITIAL HEATMAP...")
    planner.generateHeatmap(lost_people_init_loc)
    if plotBool:
        planner.showHeatmap()
    
    #print("EXECUTING SEARCHER PATHS BASED ON WAYPOINTS...")
    planner.executeSearcherPaths(waypoints)
    if plotBool:
        planner.showSearcherPaths()
        planner.showSearcherBinaryMap()

    #print("AUGMENTING HEATMAP...")
    planner.augmentHeatmap()
    if plotBool:
        planner.showHeatmap()

    #print("GENERATING DRONE PATHS...")
    fig, planner.ax = plt.subplots()
    optimalDronePaths = planner.generateDronePaths(dronePointsInit, 2)

    # Plot Final Map
    if plotBool:
        print("PLOTTING...")
        
        for path in optimalDronePaths:
            planner.plotPaths(path)
        planner.ax.imshow(planner.heatmap, cmap="hot")
        num_obstacles = len(planner.obstacles)

        for obstacle in planner.obstacles:
            coords, radius = ST.scale_obstacle(obstacle)
            circle = plt.Circle(coords, radius, color='y')
            planner.ax.add_patch(circle)
        
        plt.show()
        print("DONE.")

    duration = time.time() - st


    

    if recordTimeExecution:
        print(f"\nPath Planner took {duration} seconds to execute.")

    # Execute Drone Path Planning and measure performance...


    
    





