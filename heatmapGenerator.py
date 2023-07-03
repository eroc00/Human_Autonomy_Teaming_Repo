# Python file to generate a Missing Person Probabilistic Heatmap based on Terrain Information
from mapGenerator import SimulationMap
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt

#initial pos ranges from [-20, 20) in both x and y directions
def generateHeatmap(terrainMap:SimulationMap, initialPos, iterations=10):

    heatmap = np.ones(terrainMap.terrainHeight.shape).astype(int)

    """
    t = cv.resize(terrainMap.terrainHeight, (400, 400))
    plt.imshow(t, cmap='terrain_r')
    plt.show()

    plt.subplot(1, 3, 1)
    #plt.xlim(-200, 200)
    #plt.ylim(-200, 200)
    plt.imshow(terrainMap.terrainHeight, cmap='terrain_r')
    plt.title("Terrain")
    plt.colorbar()
    
    plt.show()
    # plt.colorbar()

    plt.subplot(1, 3, 2)
    plt.imshow(terrainMap.xGradient, cmap="gray")
    plt.title("X-Gradient")
    plt.colorbar()

    plt.subplot(1, 3, 3)
    plt.imshow(terrainMap.yGradient, cmap="gray")
    plt.title("Y-Gradient")
    plt.colorbar()

    plt.show()
    """

    x = np.array(initialPos)
    lim = terrainMap.maplen/terrainMap.res
    #print(f"Gradient at {initialPos} = {terrainMap.xGradient[x[0]][x[1]], terrainMap.yGradient[x[0]][x[1]]}")


    x1 = x.copy() # x_-1: used to calculate speed
    x0 = x.copy()
    alpha = 15
    beta = 6
    a = 1e2
    b = 1e4
    m = 70

    for t in range(iterations):
        Fr = np.random.normal(0, 1, (2))

        pos = x.astype(int)
        Fg = np.array((terrainMap.xGradient[pos[0]][pos[1]], \
                        terrainMap.yGradient[pos[0]][pos[1]]))
        
        #print(Fg)

        """ Continuous time solution to lost person model
        k = a*(np.linalg.norm(x - prevPos)) - b
        
        x =  (alpha*Fg + beta*Fr)*(m - np.exp(-k*(t+1)/m))/k 
        """

        k = a*(np.linalg.norm(x - x1)) - b

        x = (alpha*Fg + beta*Fr + (2*m + k)*x1 - m*x0 ) / (m+k)

        # Set bounds on calculations
        x = np.maximum(x, np.array([0, 0]))
        x = np.minimum(x, np.array([lim-1, lim-1]))


        #print(f"t = {t+1}: x = {x}; x-1 = {x1}; x-2 = {x0}")
        x0 = x1.copy()
        x1 = x.copy()

        # Set bounds on calculations
        pos = x.astype(int)
        
        # Add to heatmap
        heatmap[pos[0]][pos[1]] = heatmap[pos[0]][pos[1]] + 1  
    
    return heatmap
        

        # Update previous locations

def generateCHM(terrainMap:SimulationMap, initialPos, iterations=10):

    heatmap = np.ones(terrainMap.terrainHeight.shape).astype(int)

    x = np.array(initialPos)
    speed = np.array([0, 0])
    xm = [x, speed]
    lim = terrainMap.maplen/terrainMap.res

    alpha = 5
    beta = 0.3
    a = 1e3
    b = 1e5
    m = 70

    B = [[0, 0],
         [alpha/m, beta/m]]
    dt = 0.001

    for t in range(iterations):
        Fr = np.random.normal(0, 1, (2))

        pos = xm[0].astype(int)
        Fg = np.array((terrainMap.xGradient[pos[0]][pos[1]], \
                        terrainMap.yGradient[pos[0]][pos[1]]))
        F = [Fg, Fr]
        
        #print(Fg)
        
        # Update states
        k = a*(np.linalg.norm(xm[1]/dt)) - b
        A = [[0, 1],
             [0, -k/m]]

            # Stochastic Differential Equation
        dxm = np.matmul(A, xm)*dt + np.matmul(B, F)*dt
        
        speed = speed + dxm[1] # x2+ = x2 + dx2
        x = x + speed*dt # x1+ = x1 + dx1
        

        # Set bounds on calculations
        x = np.maximum(x, np.array([0, 0]))
        x = np.minimum(x, np.array([lim-1, lim-1]))

        xm = [x, speed]

        #print(f"t = {t+1}: pos = {xm[0]}; speed = {xm[1]}")

        # Set bounds on calculations
        pos = x.astype(int)
        

        # Add to heatmap
        heatmap[pos[0]][pos[1]] = heatmap[pos[0]][pos[1]] + 1

        #if not (t % 120):
            #print(f"Writing frame {t}")
            #out.write(heatmap)

    return heatmap

def LostPeopleHeatmap(terrainMap:SimulationMap, initPositions:list, iterations=25000, blur='no', blurWindow=(3, 3)):
    heatmap = np.zeros(terrainMap.terrainHeight.shape).astype(int)
    for startLoc in initPositions:
        heatmap = heatmap + generateCHM(terrainMap, startLoc, iterations=iterations)

    if blur == 'yes':
        heatmap = cv.blur(heatmap, blurWindow)

    return heatmap

    

############# TEST ################
# STATIC MAP TO BE USED IS PROVIDED IN THE FILE "TestMap.npy"

if __name__ == "__main__":

    map = SimulationMap(400, 40)
    map.loadMap()

    # Plot Terrain
    plt.rcParams["figure.figsize"] = [14.00, 7.00]
    plt.rcParams["figure.autolayout"] = True
    plt.imshow(map.terrainHeight, cmap='terrain_r')
    plt.title("Terrain")
    plt.colorbar()
    plt.show()

    lost_people_init_loc = [(5, 5)]

    heatmap = np.zeros(map.terrainHeight.shape).astype(int)

    #out = cv.VideoWriter("Heatmap_Generation.avi", cv.VideoWriter_fourcc(*"XVID"), 60, (heatmap.shape[1], heatmap.shape[0]))


    for i, startLoc in enumerate(lost_people_init_loc):
        print(f"Modeling person {i+1} starting at {startLoc}...")
        heatmap = heatmap + generateCHM(map, startLoc, iterations=25000)
    
    #out.release()

    # Plot Heatmap
    #plt.imshow(heatmap, cmap="hot")
    #plt.colorbar()
    #plt.show()

    plt.subplot(1, 3, 1)
    plt.imshow(map.terrainHeight, cmap='terrain_r')
    plt.title("Terrain")
    plt.colorbar()
    # plt.colorbar()

    plt.subplot(1, 3, 2)
    plt.imshow(heatmap, cmap="hot")
    plt.title("Raw Heatmap")
    plt.colorbar()

    # Blur is to spread out possible areas of detection so that Drone paths can detect
    # hotspots easier
    heatmap = cv.blur(heatmap, (3, 3))
    plt.subplot(1, 3, 3)
    plt.imshow(heatmap, cmap="hot")
    plt.title("Gaussian Filtered Heatmap")
    plt.colorbar()

    plt.show()