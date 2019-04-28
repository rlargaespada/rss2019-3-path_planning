import graph
import search
import numpy as np
from time import sleep

map = np.array([
    [1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0,0,0],
])

# map = np.array([
#     [0,0,0,0,0,0,0,0,0,0],
#     [0,1,1,1,1,1,1,1,1,0],
#     [0,1,0,0,0,0,0,0,1,0],
#     [0,1,0,0,0,0,0,0,1,0],
#     [0,1,0,0,0,0,0,0,1,0],
#     [0,1,0,0,0,0,0,0,1,0],
#     [0,1,0,0,0,0,0,0,1,0],
#     [0,1,0,0,0,0,0,0,1,0],
#     [0,1,1,1,1,1,1,1,1,0],
#     [0,0,0,0,0,0,0,0,0,0],
# ])
#(7, 4) {(8, 3), (4, 7), (7, 1), (4, 4), (8, 5), (7, 7), (4, 1), (8, 4)}
map = np.concatenate((np.concatenate((map, np.flipud(map)), 0), np.fliplr(np.concatenate((map, np.flipud(map)), 0))), 1)
#print(map)
start = (-0.05, 0.05) #(4,4)
goal = (-0.3, -0.2) #(9,9)
g = graph.Lookahead_Graph(start, goal, .4) #inputs should be rw
# g.test_setup((.15, .25), .05, map)
# c = g.occ_to_real_world(start)
# print('c ', c)
# r = (g.get_lookahead_neighbors(c, map))
# print(r)
# o = {g.real_world_to_occ(p) for p in r}
# print(o)
g.build_map(map, 'test_map', .05, (.15,.25,0)) #inputs should be rw
#print(g.real_world_to_occ((.15,.25)))
print(g)
print(map.size)
#print(g.get_neighbor_coords((9,9)))
#g.build_map(map)
# print(g)
#print(map.shape)
#print(g.neighbors[(2, 6)])
#print(g.neighbors[(0,0)])
# path = search.a_star(g, g.start, g.goal)
# print(path)
# p2 = [g.real_world_to_occ(p) for p in path]
# print(p2)