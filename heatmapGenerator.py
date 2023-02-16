# Python file to generate a Missing Person Probabilistic Heatmap based on Terrain Information
from mapGenerator import SimulationMap
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt

# STATIC MAP TO BE USED IS PROVIDED IN THE FILE "TestMap.npy"
map = SimulationMap(400, (10, 10))
map.loadMap()

#initial pos ranges from [-20, 20) in both x and y directions
def generateHeatmap(terrainMap:SimulationMap, initialPos, iterations=10):
    #initialPos = np.array((initialPos[0] + 20, initialPos[1] + 20))
    heatmap = np.zeros(terrainMap.terrainHeight.shape)

    #xGradient = cv.Sobel(src=terrainMap.terrainHeight, ddepth=cv.CV_64F, dx=1, dy=0)
    #yGradient = cv.Sobel(src=terrainMap.terrainHeight, ddepth=cv.CV_64F, dx=0, dy=1)

    plt.rcParams["figure.figsize"] = [14.00, 7.00]
    plt.rcParams["figure.autolayout"] = True

    plt.subplot(1, 3, 1)
    plt.imshow(terrainMap.terrainHeight, cmap='terrain_r')
    plt.title("Terrain")
    plt.colorbar()

    plt.subplot(1, 3, 2)
    plt.imshow(terrainMap.xGradient, cmap="gray")
    plt.title("X-Gradient")
    plt.colorbar()

    plt.subplot(1, 3, 3)
    plt.imshow(terrainMap.yGradient, cmap="gray")
    plt.title("Y-Gradient")
    plt.colorbar()

    plt.show()
    #cv.imshow("X Gradient", terrainMap.terrainMap.xGradient)
    #cv.imshow("Y Gradient", terrainMap.yGradient)

    #cv.waitKey()

    x = np.array((initialPos[0] + 20, initialPos[1] + 20))
    print(f"Gradient at (20, 20) = {terrainMap.xGradient[x[0]][x[1]], terrainMap.yGradient[x[0]][x[1]]}")


    x1 = x.copy() # x_-1: used to calculate speed
    x0 = x.copy()
    alpha = 5
    beta = 0.3
    a = 1e3
    b = 1e5
    m = 70

    for t in range(iterations):
        Fr = np.random.normal(0, 1, (2))
        
        Fg = np.array((terrainMap.xGradient[int(x[0])][int(x[1])], \
                        terrainMap.yGradient[int(x[0])][int(x[1])]))
        
        #print(Fg)

        """ Continuous time solution to lost person model
        k = a*(np.linalg.norm(x - prevPos)) - b
        
        x =  (alpha*Fg + beta*Fr)*(m - np.exp(-k*(t+1)/m))/k 
        """

        k = a*(np.linalg.norm(x - x1)) - b

        x = (alpha*Fg + beta*Fr + (2*m + k)*x1 - m*x0 ) / (m+k)
        print(f"t = {t+1}: x = {x}; x-1 = {x1}; x-2 = {x0}")
        x0 = x1.copy()
        x1 = x.copy()

        #x = np.rint(x)

        

        # Add to heatmap
        heatmap[int(x[0])][int(x[1])] = heatmap[int(x[0])][int(x[1])] + 1

    # Display probability map
    heatmap = heatmap / iterations
        

        # Update previous locations

    plt.imshow(heatmap, cmap="hot")
    plt.colorbar()
    plt.show()





# x = (0, 0) is the center of the map
generateHeatmap(map, (0, 0), iterations=25000)