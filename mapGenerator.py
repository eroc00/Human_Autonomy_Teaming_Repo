import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
from matplotlib.colors import LightSource
from matplotlib import cm


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

    terrainHeight = None
    mapX = None
    mapY = None
    xGradient = None
    yGradient = None
    obstacles = list()
    maplen = 0
    res = 0


    def __init__(self, size, resolution:int, minHeight = 0, maxHeight = 10, numObstacles = 6) -> None:
        self.generateMap(size, resolution, minHeight, maxHeight)

        obstacleX = size*np.random.rand(numObstacles)
        obstacleY = size*np.random.rand(numObstacles)
        obstacleRadii = 75*np.random.rand(numObstacles) + 25

        self.obstacles = list(zip(list(zip(obstacleX, obstacleY)), obstacleRadii))
        self.maplen = size
        self.res = resolution



    def adjacentNodes(self, point:tuple):
        adjNodes = []
        gridSize = int(self.maplen/self.res)

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
    def generateMap(self, size:int, resolution:int, minHeight = 0, maxHeight = 15):
        i = resolution # resolution, 10x10 according to ryan williams' paper
        j = resolution
        
        range = int(size/2)

        x = np.linspace(-range, range, int(size/i))
        y = np.linspace(-range, range, int(size/j))

        self.mapX, self.mapY = np.meshgrid(x, y)

        # Variables extrapolated from Path Planning Paper's example terrain
        randNumMin = 0

        # Generate a map of size self.mapX.shape filled with random numbers from a normal distribution.
        # For all entries in the initial map, limit the lowest value to be randNumMin
        initialMap = np.maximum(np.random.normal(0, 1, self.mapX.shape), randNumMin)
        print(f"Initial Map's dimensions: {initialMap.shape}, Max Height: {initialMap.max()}, Min Height = {initialMap.min()}")


        # Perform an operation on all values of initialMap to translate from range [randNumMin, 1], to [minHeight, maxHeight]
        self.terrainHeight = self.convertRange(initialMap, initialMap.min(), initialMap.max(), minHeight, maxHeight)

        print(f"Terrain Height's dimensions: {self.terrainHeight.shape}, Max Height: {np.max(self.terrainHeight)}, Min Height = {np.min(self.terrainHeight)}")


        # Smoothen map by applying a 2D gaussian filter
        self.terrainHeight = cv.blur(self.terrainHeight, (3, 3))

        # Generate gradients
        self.xGradient = cv.Sobel(src=self.terrainHeight, ddepth=cv.CV_64F, dx=1, dy=0)
        self.yGradient = cv.Sobel(src=self.terrainHeight, ddepth=cv.CV_64F, dx=0, dy=1)

    def saveMap(self, mapName:str = "TestMap"):
        np.save(mapName, self.terrainHeight)
        with open(f"{mapName}_obstacles.txt", 'w') as fp:
            for obj in self.obstacles:
                fp.write("%f %f, %f\n" % (obj[0][0], obj[0][1], obj[1]))


    # Load function is designed to read 400x400 maps with a 10x10 resolution. 
    # WILL NOT WORK FOR ANY OTHER MAP SPEC UNLESS EDITED
    def loadMap(self, mapName:str="TestMap.npy"):
        self.maplen = 400
        self.res = 10

        range = int(self.maplen/2)
        x = np.linspace(-range, range, int(self.maplen/self.res))
        y = np.linspace(-range, range, int(self.maplen/self.res))

        self.terrainHeight = np.load(mapName)
        self.xGradient = cv.Sobel(src=self.terrainHeight, ddepth=cv.CV_64F, dx=1, dy=0)
        self.yGradient = cv.Sobel(src=self.terrainHeight, ddepth=cv.CV_64F, dx=0, dy=1)
        self.mapX, self.mapY = np.meshgrid(x, y)
        obstacles = list()

        with open(f"{mapName.replace('.npy', '')}_obstacles.txt", 'r') as fp:
            for line in fp:

                data = line[:-1]
                position, radii = data.split(', ')
                x, y = position.split(' ')

                obstacles.append([(float(x), float(y)), float(radii)])
                



    ###### VISUALIZE MAP ######
    def plot(self, plotMaxHeight=80, rstride=2, cstride=2):
        #maplen, mapwidth = map.shape # map size

        print(self.mapX.size, self.mapY.size, self.terrainHeight.size)
        # Plot a basic wireframe.        
        # axis limits
        #plt.xlim(0, 400)
        #plt.ylim(0, 400)
        fig, ax = plt.subplots(subplot_kw=dict(projection='3d'))

        ls = LightSource(0, 0)
        # To use a custom hillshading mode, override the built-in shading and pass
        # in the rgb colors of the shaded surface calculated from "shade".
        rgb = ls.shade(self.terrainHeight, cmap=cm.gist_earth, vert_exag=0.1, blend_mode='soft')
        surf = ax.plot_surface(self.mapX, self.mapY, self.terrainHeight, rstride=rstride, cstride=cstride, facecolors=rgb) #,\
                            #linewidth=0, antialiased=False, shade=False)
        #ax.plot_wireframe(terrainMap.mapX, terrainMap.mapY, terrainMap.terrainHeight, rstride=1, cstride=1)
        ax.set_zlim(0, 5)
        plt.title("Terrain")
        plt.show()

    def convertRange(self, x, old_min, old_max, new_min, new_max):
        # normalize x to the range [0, 1]
        normalized_x = (x - old_min) / (old_max - old_min)
        
        # scale x to the new range
        new_x = (normalized_x * (new_max - new_min)) + new_min
        
        return new_x

# Test


if __name__ == "__main__":
    map = SimulationMap(400, 10, maxHeight=3)
    map.loadMap()
    map.plot(plotMaxHeight = 15, rstride=1, cstride=1)
    prompt = input("Would you like to save the map? ")
    if prompt.lower() == 'yes':
        map.saveMap()
        print("Map Saved.")