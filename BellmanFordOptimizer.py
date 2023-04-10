from searcherTrajectories import a_star_uav, coverage, plot_paths, scale_obstacle
from mapGenerator import SimulationMap
from heatmapGenerator import LostPeopleHeatmap
from itertools import product
import matplotlib.pyplot as plt
import math

# Algorithm: https://en.wikipedia.org/wiki/Bellman%E2%80%93Ford_algorithm

plotBool = False

class BellmanFord():
    def __init__(self) -> None:
        if plotBool:
            fig, self.ax = plt.subplots()
            self.ax.set_aspect('equal')
            pass

    def initGraph(self, map:SimulationMap, startPoint, endPoint, midWaypointsNum, xRange):
        self.distance = None
        self.predecessor = None
        if (endPoint[1] - startPoint[1]) < 0:
            self.startPoint = endPoint
            self.endPoint = startPoint
            height = startPoint[1] - endPoint[1]
        else:
            self.startPoint = startPoint
            self.endPoint = endPoint 
            height = endPoint[1] - startPoint[1]
        self.map = map
        self.V = []
        self.E = set()
        self.C = {}
        
        self.V = [[self.startPoint]]
        
        yDivisions = height/(midWaypointsNum+1)

        # Neglecting case where start point begins at a higher y value than end point

        waypoints = [(0, int(i*yDivisions)) for i in range(1, midWaypointsNum+1)]

        l = 0
        for vert in waypoints:
            layer = self.V[l]

            # Create vertices int(xRange(1) - xRange(0))
            next_Layer =[(i, vert[1]) for i in range(xRange[0], xRange[1])]
            next_Layer = [point for point in next_Layer if not self.isObstructed(point, map.obstacles)]

            self.V.append(next_Layer)

            # Add edges
            edgeList = []
            for node0 in layer:
                for node1 in next_Layer:
                    edgeList.append((node0, node1))

            self.E = self.E.union(edgeList)

            l += 1

        # Join last edges to end vertex
        layer = self.V[-1]
        edgeList = [(node0, self.endPoint) for node0 in layer]
        self.E = self.E.union(edgeList)

        self.C = {edge: self.cost(edge) for edge in self.E}

        # Collapse list of lists 
        self.V = [item for sublist in self.V for item in sublist]
        self.V.append(self.endPoint)

    def optimize(self):
        # Step 1: Initialize graph
        self.distance = {node: float('inf') for node in self.V}
        self.predecessor = {node: None for node in self.V}
        self.distance[self.startPoint] = 0

        # Step 2: Relax edges repeatedly
        for i in range(len(self.V)-1):
            for edge in self.E:
                u = edge[0]
                v = edge[1]

                # Negating the weights is important to find the 'longest' path
                # As long as the initial weights are never negative
                w = -self.C[edge]
                if self.distance[u] + w < self.distance[v]:
                    self.distance[v] = self.distance[u] + w
                    self.predecessor[v] = u

    def optimalWaypoints(self):
        optimalWaypoints = []

        backtrack = self.endPoint
        optimalWaypoints.append(backtrack)
        while self.predecessor[backtrack] != None:
            backtrack = self.predecessor[backtrack]
            optimalWaypoints.append(backtrack)

        self.waypoints = optimalWaypoints[::-1]

    def isObstructed(self, point, obstacles):
        for obstacle in obstacles:
            ob_cords, ob_radius = scale_obstacle(obstacle)
            if math.dist(point, ob_cords) <= ob_radius:
                return True
            return False


    def optimalPath(self):
        self.optimalWaypoints()
        optimizedWaypoints = self.waypoints

        start = optimizedWaypoints.pop(0)
        optimizedPath = a_star_uav(self.map, self.map.heatmap, optimizedWaypoints, start[0], start[1])

        return optimizedPath
    
    def cost(self, edge):
        start = edge[0]
        end = edge[1]

        path = a_star_uav(self.map, self.map.heatmap, [end], start[0], start[1])
        
        path_heat, _, _ = coverage(self.map, path, self.map.heatmap)

        # NEED TO CONSIDER PATH LENGTH; ANALOGOUS TO BATTERY LIFE

        return path_heat
    
    def plotWaypoints(self):
        x_coords = [point[0] for point in self.waypoints]
        y_coords = [point[1] for point in self.waypoints]

        self.ax.scatter(x_coords, y_coords, 200)

    def plot_obstacles(self, map):
        num_obstacles = len(map.obstacles)
        #fig, ax = plt.subplots()
        #plt.xlim(0, 40)
        #plt.ylim(0, 40)
        for i in range(num_obstacles):
            obstacle = map.obstacles[i]
            cords, radius = scale_obstacle(obstacle)
            # print(cords, radius)
            circle = plt.Circle(cords, radius, color='y')
            self.ax.add_patch(circle)



def cost(edge, heatmap):
    start = edge[0]
    end = edge[1]

    path = a_star_uav(map, heatmap, [end], start[0], start[1])
    
    path_heat, _, _ = coverage(map, path, heatmap)

    return path_heat, path
    
def rawRun(map):
     #Map Initialization
    
    #lost_people_init_loc = [(20, 20), (12, 8), (10, 27), (28, 10), (26, 20)]
    lost_people_init_loc = [(5, 5), (3, 2), (2, 6), (7, 2), (6, 4)]

    heatmap = LostPeopleHeatmap(map, lost_people_init_loc)

    fig, ax = plt.subplots()
    plt.imshow(map.terrainHeight, cmap='terrain_r')
    ax.imshow(heatmap, cmap="hot")

    plt.show()


    start = (5, 0)
    initial_vert = [(5, 3), (5, 7)]
    end = (5, 9)

    width = map.maplen/map.res

    V = [[start]]
    E = set()

    l = 0
    for vert in initial_vert:
        layer = V[l]

        # Create vertices
        next_Layer =[(i, vert[1]) for i in range(int(width))]
        V.append(next_Layer)

        # Add edges
        edgeList = []
        for node0 in layer:
            for node1 in next_Layer:
                edgeList.append((node0, node1))

        E = E.union(edgeList)

        l += 1

    # Join last edges to end vertex
    layer = V[-1]
    edgeList = [(node0, end) for node0 in layer]
    E = E.union(edgeList)

    C = {edge: cost(edge, heatmap) for edge in E}

    # Collapse list of lists 
    V = [item for sublist in V for item in sublist]
    V.append(end)


    ## START BELLMAN-FORD ALGORITHM

    # Step 1: Initialize graph
    distance = {node: float('inf') for node in V}
    predecessor = {node: None for node in V}
    distance[start] = 0

    # Step 2: Relax edges repeatedly
    for i in range(len(V)-1):
        for edge in E:
            u = edge[0]
            v = edge[1]

            # Negating the weights is important to find the 'longest' path
            # This works as long as the initial weights (C) are never negative
            w = -C[edge][0]
            if distance[u] + w < distance[v]:
                distance[v] = distance[u] + w
                predecessor[v] = u

    #print(predecessor)

    optimizedPath = []

    backtrack = end
    optimizedPath.append(backtrack)
    while predecessor[backtrack] != None:
        backtrack = predecessor[backtrack]
        optimizedPath.append(backtrack)
        

    # Reverse list
    optimizedWaypoints = optimizedPath[::-1]

    start = optimizedWaypoints.pop(0)
    optimizedPath = a_star_uav(map, heatmap, optimizedWaypoints, start[0], start[1])

    # Plot Results
    plt.imshow(heatmap, cmap="hot")
    x_path = [point[0] for point in optimizedPath]
    y_path = [point[1] for point in optimizedPath]

    plot_paths(x_path, y_path, optimizedPath, start[0], start[1], 'b')
    
        # Plot Optimized Waypoints
    x_coords = [point[0] for point in optimizedWaypoints]
    y_coords = [point[1] for point in optimizedWaypoints]
    plt.scatter(x_coords, y_coords, 200)


    plt.show()

    print(f"Optimal Waypoints: {optimizedWaypoints}\n")
    print(f"Optimal Path: {optimizedPath}\n")

def SARRun(map):

    map.loadMap()

    lost_people_init_loc = [(20, 20), (12, 8), (10, 27), (28, 10), (26, 20)]
    map.heatmap = LostPeopleHeatmap(map, lost_people_init_loc)
    

    gridlen = int(map.maplen/map.res)-1
    
    domain = (0, gridlen)

    mapSegments = 3
    segmentLen = int(gridlen/mapSegments)
    pathOptimizationInputs = []
    for i in range(mapSegments):
        
        midPoint = int(i*segmentLen + segmentLen/2)
        start = (midPoint, 0)
        end = (midPoint, gridlen)
        domain = (i*segmentLen, (i+1)*segmentLen)

        pathOptimizationInputs.append([start, end, 2, domain])

    # Change which segment you want to plan the paths for
    input = pathOptimizationInputs[2]
    start = input[0]
    end = input[1]

    optimizer = BellmanFord()
    optimizer.initGraph(map, start, end, input[2], input[3])
    optimizer.optimize()

    path = optimizer.optimalPath()

    # Plot results
    optimizer.plot_obstacles(map)

    optimizer.ax.imshow(map.heatmap, cmap="hot")
    x_path = [point[0] for point in path]
    y_path = [point[1] for point in path]
    plot_paths(x_path, y_path, path, start[0], start[1], 'b')
    optimizer.plot_obstacles(map)

    optimizer.plotWaypoints()
    print(optimizer.waypoints)

    plt.show()
    


if __name__ == "__main__":
    map = SimulationMap(400, 40, maxHeight=3, numObstacles=0)

    rawRun(map)
    #SARRun(map)

    
   