from math import floor

# class Node(object):
# 	def __init__(self,x,y, val=0):
# 		self.position = (x,y)
# 		self.value = val

# class Dimensioned_Node(Node):
# 	#dimensions should be odd numbers
# 	def __init__(self, x, y, val, dim):
# 		Node.__init__(self, x, y, val)
# 		self.dim = dim
# 		b = floor(dim/2)
# 		self.bounds = {(self.position[0]-b, self.position[1]-b), (self.position[0]-b, self.position[1]+b),
# 						(self.position[0]+b, self.position[1]-b), (self.position[0]+b, self.position[1]+b)}

class Graph(object):
	def __init__(self, start, goal):
		self.nodes = set()
		self.neighbors = {} # dict of node mapped to edges
		self.start = start
		self.goal = goal

	def cost(self, p1, p2):
		return ((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)**(0.5)

	def add_node(self, node):
		if node in nodes:
			return
		self.nodes.add(node)
		self.neighbors[node] = []

	def add_edge(self, node1, node2):
		self.neighbors[node1].append(node2)

	def __str__(self):
		p = dict()
		for node in self.nodes:
			p[node] = []
			for n in self.neighbors[node]:
				p[node].append(n)
		for node in p:
			print(node, ':', p[node])

		return ''

	def get_neighbor_coords(self,coord):
		x = coord.position[0]
		y = coord.position[1]

		neighbors = {(x-1, y-1), (x-1, y), (x-1, y+1),
					(x, y-1),(x, y+1),
					(x+1, y-1),(x+1, y),(x+1, y+1)}

		return neighbors

	def build_map(self, map):
		x_max = map.shape[0]
		y_max = map.shape[1]

		for x in range(x_max):
			for y in range(y_max):
				if map[x,y] != 1:
					pos = (x,y)
					self.add_node(pos)
					for coord in self.get_neighbor_coords(pos):
						if 0 <= coord[0] <= x_max and 0 <= coord[1] <= y_max:
							self.add_node(coord)
							self.add_edge(pos, coord)

							
	def build_consolidated_map(self, map):
		x_max = map.shape[0]
		y_max = map.shape[1]

		g = Consolidated_Graph(3, 5)

		occupied_coords = set()

		#find all ones
		for x in range(x_max):
			for y in range(y_max):
				if map[x,y] == 1:
					occupied_coords.add((x, y))

		#squares next to ones should be single-square zeros
		for square in occupied_coords:
			neighbors = g.get_neighbor_coords(square)
			for i in range(3):
				if map[neighbors[i][0], neighbors[i][0]] != 1:
					pass

		#open spaces next to those should be medium-sized squares

		#open speces next to those should be large size squares


	def heuristic(self, node1):
		point1 = node1
		point2 = self.goal
		return ((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)**(0.5)

class Consolidated_Graph(Graph):
	def __init__(self, med, large):
		Graph.__init__(self)
		self.med_dim = med
		self.large_dim = large

	def get_neighbor_coords(self,coord):
		x = coord[0]
		y = coord[1]

		#U, L, R, D
		neighbors = [(x-1, y),
					(x, y-1),(x, y+1),
					(x+1, y)]

		return neighbors

	def build_consolidated_map(self, map):

		def is_clear(size, direction):
			pass

		def medium_square(coor, direction):
			pass

		def large_square(coor, direction):
			pass
		
