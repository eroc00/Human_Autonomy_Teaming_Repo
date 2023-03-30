import math
import numpy as np
from random import random
from operator import length_hint
from mapGenerator import SimulationMap
import matplotlib as mpl
import matplotlib.pyplot as plt




class UAVTrajectories:
    ''' 
    Python File to generate UAV Trajectories based on Terrain Information
    gamma > 0, zeta > 0, eta> 0 -- tunable parameters to adjust the radius of the D-dimensional sphere
    r                           -- radius of D-dimensional sphere
    |V|                         -- the cardinality of the set of vertices V
    V                           -- Vertices (V <-Xstart)
    X                           -- configuration space 
    Xobstacles                  -- obstacles (within X)
    Xfree                       -- free space ( xfree = X/ Xobstacle)
    Xstart                      -- start configuration (within xfree)
    Xgoal                       -- goal configuration (within xfree)
    E                           -- edges (E <- 0)
    Xrand                       -- random samples (within X)
    '''

    def __init__(self, size, resolution) -> None:
        self.resolution = resolution
        self.gridSize = size
     
    def RRT(self):
        #X = SimulationMap(400, (10, 10))
        #map.loadMap()
        #map.plot()

        

        #Xfree = X / Xobstacle
        Xstart = (10, 0)
        Xgoal  = (10, 40)
        Xobstacle, Xcenters, Xradii = self.generateObstacles(10)

        OK = False
        while not OK:
            START_COLLISIONS = self.obstacle_check(Xstart, Xobstacle)    # check for collision between start point and obstacles
            END_COLLISIONS   = self.obstacle_check(Xgoal, Xobstacle)     # check for collision between end point and obstacles

            if (any(START_COLLISIONS)) or (any(END_COLLISIONS)):         # if a collision with any obstacle exists
                pass                                                     # do nothing; loop will re-run
            else:                                                        # if no collision exists
                OK = True                                                # clear to move forward
        
        V = [['q0', Xstart, 'None']]
        E = []

        CLEAR_PATH = False
        #POINT_OK = False
        
        while not CLEAR_PATH:
            POINT_OK = False
            # generate new node and check for collisions with obstacles
            while not POINT_OK:
                POINT_NAME = "q{}".format(len(V))
                # coordinates of random point -- Xrand = self.randomPosition()
                POINT_X = np.random.randint(0, 101)
                POINT_Y = np.random.randint(0, 101)
                Xrand = (POINT_X, POINT_Y)

                # determine node closest to random point, distance to it, and its list index 
                distChange = 10e10
                #Xnearest = self.getNearestVertex(Xrand)
                for i in range(len(V)):                                                 # iterate through existing nodes
                    DIST_TO_POINT = self.distance(V[i][1], Xrand)                       # calculate distance to each node
                    if DIST_TO_POINT <= distChange:                                       # if distance to node is less than minimum distance
                        distChange = DIST_TO_POINT                                        # save distance to node as minimum distance
                        CLOSEST_NODE = i                                                # save list index of closest node

                PARENT = "q{}".format(CLOSEST_NODE)         # "parent" node name
                PARENT_COORDS = (V[CLOSEST_NODE][1])        # "parent" node coordinates

                # create vector from closest node to random point
                D_X = POINT_X - V[CLOSEST_NODE][1][0]
                D_Y = POINT_Y - V[CLOSEST_NODE][1][1]
                SHORTEST_VECTOR = [D_X, D_Y]
                # magnitude of vector to closest node
                SHORTEST_MAG = math.sqrt((D_X**2)+(D_Y**2))
                # calculate unit vector
                UNIT_VECTOR = [dist/SHORTEST_MAG for dist in SHORTEST_VECTOR]

                # coordinates of new node
                NODE_X = V[CLOSEST_NODE][1][0] + UNIT_VECTOR[0]
                NODE_Y = V[CLOSEST_NODE][1][1] + UNIT_VECTOR[1]
                Node = (NODE_X, NODE_Y)

                # check for collisions with obstacles
                NODE_COLLISIONS = self.obstacle_check(Node, Xobstacle)

                if any(NODE_COLLISIONS):                    # if collision exists
                    pass                                    # do nothing and re-run loop
                else:                                       # if no collisions
                    POINT_OK = True                         # move forward

            # add new node data to list
            V.append([POINT_NAME, Node, PARENT])

            # add new path segment to list
            E.append([PARENT_COORDS, Node])

            # determine if line to endpoint intersects obstacles
            LINE_COLLISIONS = []        # list of obstacle collision conditions
            for j in range(10):
                TOO_CLOSE = False       # obstacle intersection condition
                BETWEEN = False         # intersection between endpoints condition

                # get intersection length ratio and distance to obstacle center
                RATIO_U, DIST_TO_OBSTACLE = self.point2line_params(V[-1][1], Xgoal, Xobstacle[0][j])

                # intersection with obstacle is between node and endpoint if 0 < u < 1
                if 0 <= RATIO_U <= 1:
                    BETWEEN = True

                # line intersects obstacle if distance to obstacle center < obstacle radius
                if DIST_TO_OBSTACLE <= Xobstacle[1][j]:
                    TOO_CLOSE = True

                # if line to endpoint intersects obstacle between node and endpoint, path is blocked
                if BETWEEN and TOO_CLOSE:
                    LINE_COLLISIONS.append(True)        # add obstacle collision condition to list
                else:
                    LINE_COLLISIONS.append(False)

            if any(LINE_COLLISIONS):                    # if any obstacle collisions exist
                pass                                    # re-run loop to generate another node
            else:                                       # if no collisions exist
                CLEAR_PATH = True                       # move forward


        # generate final path segment & add to list
        E.append([V[-1][1], Xgoal])

        # work backwards to identify successful path
        PATH_POINTS = [V[-1][1]]                        # nodes in successful path
        PATH_SEGS = [[V[-1][1], Xgoal]]                 # start path by adding final segment
        AT_START = False                                # condition for reaching start point
        CURRENT_COORDS = V[-1][1]                       # x-y coordinates of last point in path
        PARENT_NODE = V[-1][2]                          # name of initial parent node
        while not AT_START:                                            # iterate until start is reached
            for l in range(len(V)):                                    # scan all nodes to find parent
                if V[l][0] == PARENT_NODE:                             # if node name matches parent node name
                    PATH_POINTS.insert(0, V[l][1])                     # add node to path
                    PATH_SEGS.insert(0, [V[l][1], CURRENT_COORDS])     # add path segment from node to parent
                    CURRENT_COORDS = V[l][1]                           # parent node becomes new current node
                    PARENT_NODE = V[l][2]                              # new parent node
                    print(PARENT_NODE)
            if PARENT_NODE == 'None':                                  # once start point is reached
                AT_START = True                                        # move forward
        # create node plot points
        NODES_PLOT_X = [V[a][1][0] for a in range(len(V))]
        NODES_PLOT_Y = [V[a][1][1] for a in range(len(V))]
        
        # create path plot points
        PATH_PLOT_X = [PATH_POINTS[b][0] for b in range(len(PATH_POINTS))]
        PATH_PLOT_Y = [PATH_POINTS[b][1] for b in range(len(PATH_POINTS))]

        # plot colors
        START_COLOR = '#5dc91f'             # start point color
        END_COLOR = '#ff9b00'               # end point color
        POINTS_COLOR = '#1691e5'            # color for all nodes/edges
        PATH_COLOR = '#c0120a'              # color of path nodes and segments

        # create image
        FIG, AX = plt.subplots(num = "Pathfinding RRT", nrows=1, ncols=1, sharex=True, sharey=True, figsize=(9, 9))
        # axis limits
        plt.xlim(0, 40)
        plt.ylim(0, 40)

        # plot obstacles as circular patch collection
        OBSTACLES = [plt.Circle(center, radius) for center, radius in zip(Xcenters, Xradii)]
        PATCHES = mpl.collections.PatchCollection(OBSTACLES, facecolors='black')
        AX.add_collection(PATCHES)
        # plot start & end points
        plt.scatter(Xstart[0], Xstart[1], s=100, c=START_COLOR, marker='+')
        plt.scatter(Xgoal[0], Xgoal[1], s=100, c=END_COLOR, marker='+')

        # plot all nodes/edges one by one
        for i in range(len(V)):
            # plot newest node
            plt.scatter(NODES_PLOT_X[i], NODES_PLOT_Y[i], s=10, c=POINTS_COLOR)
            if i > 0:
                # plot newest edge
                NODE_SEGMENTS = mpl.collections.LineCollection(E[i-1:i], colors=POINTS_COLOR)
                AX.add_collection(NODE_SEGMENTS)
            plt.pause(0.1)

        # plot final segment
        NODE_SEGMENTS = mpl.collections.LineCollection([E[-1], [Xgoal]], colors=POINTS_COLOR)
        AX.add_collection(NODE_SEGMENTS)
        plt.pause(0.1)

        for i in range(len(PATH_POINTS)):
            # plot newest path node
            plt.scatter(PATH_PLOT_X[i], PATH_PLOT_Y[i], s=10, c=PATH_COLOR)
            if i > 0:
                # plot newest edge
                PATH_SEGMENTS = mpl.collections.LineCollection(PATH_SEGS[i-1:i], colors=PATH_COLOR)
                AX.add_collection(PATH_SEGMENTS)
            plt.pause(0.05)

        # plot final path segment
        PATH_SEGMENTS = mpl.collections.LineCollection([PATH_SEGS[-1], [Xgoal]], colors=PATH_COLOR)
        AX.add_collection(PATH_SEGMENTS)
        plt.pause(0.05)

        # show image
        plt.show()              

        """
        for i in Nsamples:
            Xrand = self.randomPosition()
            Xnearest = self.getNearestVertex(Xrand)
            Xnew = self.steer(Xnearest, Xrand, distChange, maxDist)
            E += {(Xnearest, Xnew)}
            r = min(((self.gamma/self.zeta) * (math.log(Vabs)/Vabs)**(1/D)),eta)
            
            j = self.getVerticesInBallOfRadius(Xnew, r)/ Xnearest

            for Xnbor in j:
                if self.cost(Xnbor) > (self.cost(Xnew) + self.distance(Xnew,Xnbor)):
                    E = E/(self.parentVertexOf(Xnearest), Xnearest)
                    E += {(Xnew, Xnbor)}
        """

    #Returns length between two points
    def distance(self, point1:tuple, point2:tuple):
        length = math.sqrt((point2[0]-point1[0])**2 + (point2[1]-point1[1])**2)
        return length

    def generateObstacles(self, N):
        #Generate Random Obstacles
        OBSTACLE_PROPS = []                 # list to hold obstacle centers/radii
        OBSTACLE_CENTER = []                # list of obstacle center coordinates as tuples
        OBSTACLE_RADII = []
        # x- and y-coordinates of obstacles
        for i in range(N):
            OBSTACLE_X = 40*np.random.rand(1)
            OBSTACLE_Y = 40*np.random.rand(1)
            
            OBSTACLE_CENTER.append([OBSTACLE_X, OBSTACLE_Y])
            #obstacle radii
            OBSTACLE_RADII.append(5*np.random.rand(1))
        # fill obstacle properties list
        OBSTACLE_PROPS = [OBSTACLE_CENTER, OBSTACLE_RADII]

        return OBSTACLE_PROPS, OBSTACLE_CENTER, OBSTACLE_RADII


    def obstacle_check(self, p_check, OBSTACLE_PROPS):
        """
            Checks whether a point is inside any of the obstacles.
            ARGUMENTS
                p_check         Point to check; tuple
            OUTPUT
                in_obstacles    Obstacle collision conditions; list of Boolean values
        """
        in_obstacles = []                   # intersection condition (Boolean) with each obstacle
        for ind_a in range(10):              # for each obstacle
            # calculate distance from point to obstacle center
            distance_to = self.distance(OBSTACLE_PROPS[0][ind_a], p_check)
            # check distance against obstacle radius
            #print("Distance: ", distance_to, "   Radius: ",OBSTACLE_PROPS[1][ind_a] )
            if distance_to <= OBSTACLE_PROPS[1][ind_a]:     # if radius > distance
                in_obstacles.append(True)                   # mark point within obstacle
            else:                                           # if distance > radius
                in_obstacles.append(False)                  # mark point outside obstacle

        #print(in_obstacles)

        return in_obstacles

    def point2line_params(self, p_1, p_2, p_3):
        """
            Defines a line passing through two points. Determines whether the tangent to
            the line passing through a third point intersects between the first two. Formula is
            defined at http://paulbourke.net/geometry/pointlineplane/
            ARGUMENTS
                p_1     Point 1 coordinates; tuple
                p_2     Point 2 coordinates; tuple
                p_3     Point 3 coordinates; tuple
            OUTPUT
         """
        seg_vec = (p_2[0] - p_1[0], p_2[1] - p_1[1])                # P1 -> P2 vector
        seg_mag = math.sqrt(seg_vec[0]**2 + seg_vec[1]**2)          # length of P1 -> P2 segment

        # determine intersection length ratio u
        u = ((p_3[0] - p_1[0])*(p_2[0] - p_1[0]) + (p_3[1] - p_1[1])*(p_2[1] - p_1[1]))/(seg_mag**2)

        # coordinates of intersection point
        p_x = p_1[0] + u*(p_2[0] - p_1[0])
        p_y = p_1[1] + u*(p_2[1] - p_1[1])

        # distance from P3 to intersection point
        distance = self.distance((p_x, p_y), p_3)

        return u, distance


#Testing
test = UAVTrajectories(400, (10,10))
test.RRT()

'''
# STATIC MAP TO BE USED IS PROVIDED IN THE FILE "TestMap.npy"
map = SimulationMap(400, (10, 10))
map.loadMap()
map.plot()
'''