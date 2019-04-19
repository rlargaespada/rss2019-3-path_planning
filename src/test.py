import graph
import search
import numpy as np

map = np.array([
    [0,0,0,0,0,0,0,0,0,0],
    [0,0,1,1,1,1,1,1,0,0],
    [0,0,0,0,0,0,0,1,0,0],
    [0,0,0,0,0,0,0,1,0,0],
    [0,0,0,0,0,0,0,1,0,0],
    [0,0,0,0,0,0,0,1,0,0],
    [0,0,0,0,0,0,0,1,0,0],
    [0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,0,0]
])
start = (9,0)
goal = (0,9)
g = graph.Graph(start, goal)
#print(g.get_neighbor_coords((9,9)))
g.build_map(map)
#print(map.shape)
#print(g.neighbors[(2, 6)])
#print(g.neighbors[(0,0)])
path = search.a_star(g, start, goal)
print(path)