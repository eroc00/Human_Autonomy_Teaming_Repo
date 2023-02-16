import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv



class SimulationMap:
    """This class represents the terrain in which we develop and test our path planning algorithms
         Member variables:
            terrainHeight - a Matrix which holds the terrain height at a given point (x1, x2) on the map
                Example, self.terrainHeight[2, 4] yields a height in meters at location (2, 4)
            xGradient - tendency to move left or right in the terrain given a point (x1, x2)
            yGradient - tendency to move up or down in the terrain given a point (x1, x2)            
            mapX - a collection of x coordinates (in meters) represented as a Matrix. Mainly used for visualization
            mapY - a collection of y coordinates (in meters) represented as a Matrix. Mainly used for visualization

    """

    def __init__(self, size, resolution) -> None:
        self.generateMap(size, resolution)


    def adjacentNodes(self, point:tuple, gridSize):
        adjNodes = []

        if (point[0] < 0 or point[0] > gridSize-1 or point[1] < 0 or point[1]>gridSize-1):
            return adjNodes

        for x in range(3):
            for y in range(3):
                xpos = point[0] + x-1
                ypos = point[1] + y-1

                # IF out of bounds OR current location, DO NOT ADD TO LIST
                if (xpos < 0 or xpos > gridSize-1 or ypos < 0 or ypos>gridSize-1) or (xpos == point[0] and ypos == point[1]):
                    continue

                else:
                    adjNodes.append((xpos, ypos))

        return adjNodes

    # Generates a square map with side lengths 'side' and stores relevant data to class' terrainHeight, mapX, and mapY
    # Inputs:
    #   size - side lengths of map
    #   resolution - coordinate increment rate
    #   minHeight - minimum land height found in map at any point
    #   maxHeight - maximum land height found in map at any point
    def generateMap(self, size:int, resolution:tuple, minHeight = 0, maxHeight = 15):
        i, j = resolution # resolution, 10x10 according to ryan williams' paper

        range = int(size/2)

        x = np.linspace(-range, range, int(size/i))
        y = np.linspace(-range, range, int(size/j))

        self.mapX, self.mapY = np.meshgrid(x, y)

        # Variables extrapolated from Path Planning Paper's example terrain
        maxHeight = 15 #meters
        minHeight = 0 #meters
        randNumMin = 0

        # Generate a map of size self.mapX.shape filled with random numbers from a normal distribution.
        # For all entries in the initial map, limit the lowest value to be randNumMin
        initialMap = np.maximum(np.random.normal(0, 0.34, self.mapX.shape), randNumMin)
        print(f"Initial Map's dimensions: {initialMap.shape}")


        # Perform an operation on all values of initialMap to translate from range [randNumMin, 1], to [minHeight, maxHeight]
        self.terrainHeight = (initialMap - randNumMin)*(maxHeight - minHeight)/(1 - randNumMin) + minHeight

        # Smoothen map by applying a 2D gaussian filter
        self.terrainHeight = cv.blur(self.terrainHeight, (3, 3))

        # Generate gradients
        self.xGradient = cv.Sobel(src=self.terrainHeight, ddepth=cv.CV_64F, dx=1, dy=0)
        self.yGradient = cv.Sobel(src=self.terrainHeight, ddepth=cv.CV_64F, dx=0, dy=1)

    def saveMap(self, mapName:str):
        np.save(mapName, self.terrainHeight)

    # Load function is designed to read 400x400 maps with a 10x10 resolution. 
    # WILL NOT WORK FOR ANY OTHER MAP SPEC UNLESS EDITED
    def loadMap(self, mapName:str="TestMap.npy"):
        sidelen = 400
        res = 10

        range = int(sidelen/2)
        x = np.linspace(-range, range, int(sidelen/res))
        y = np.linspace(-range, range, int(sidelen/res))

        self.terrainHeight = np.load(mapName)
        self.xGradient = cv.Sobel(src=self.terrainHeight, ddepth=cv.CV_64F, dx=1, dy=0)
        self.yGradient = cv.Sobel(src=self.terrainHeight, ddepth=cv.CV_64F, dx=0, dy=1)
        self.mapX, self.mapY = np.meshgrid(x, y)



    ###### VISUALIZE MAP ######
    def plot(self, plotMaxHeight=80, rstride=2, cstride=2):
        #maplen, mapwidth = map.shape # map size

        Z = self.terrainHeight.copy()

        Z[1, 1] = plotMaxHeight

        print(self.mapX.size, self.mapY.size, Z.size)

        # Plot a basic wireframe.
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.plot_wireframe(self.mapX, self.mapY, Z, rstride=rstride, cstride=cstride)

        plt.show()

# Test

map = SimulationMap(400, (10, 10))
map.loadMap()
#map.plot()