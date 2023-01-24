# Python file to develop Initial Robot Paths based on Terrain Information and eventually Searcher Trajectories

from mapGenerator import SimulationMap

# STATIC MAP TO BE USED IS PROVIDED IN THE FILE "TestMap.npy"
map = SimulationMap(100, (10, 10))
map.loadMap()