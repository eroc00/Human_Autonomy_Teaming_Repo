# Python file to generate a Missing Person Probabilistic Heatmap based on Terrain Information
from mapGenerator import SimulationMap

# STATIC MAP TO BE USED IS PROVIDED IN THE FILE "TestMap.npy"
map = SimulationMap(400, (10, 10))
map.loadMap()