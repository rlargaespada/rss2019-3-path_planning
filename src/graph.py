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

class Waypoint(object):
	def __init__(self, x, y, theta):
		self.x = x
		self.y = y
		self.theta = theta

class Graph(object):
	def __init__(self, start, goal):
		self.nodes = set()
		self.neighbors = {} # dict of node mapped to edges
		self.start = start
		self.goal = goal

	def cost(self, p1, p2):
		return ((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)**(0.5)

	def add_node(self, node):
		if node in self.nodes:
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
		x = coord[0]
		y = coord[1]

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


	def heuristic(self, node1):
		point1 = node1
		point2 = self.goal
		return ((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)**(0.5)

class Consolidated_Graph(Graph):
	def __init__(self, start, goal, med, large):
		Graph.__init__(self, start, goal)
		# set of occupied coordinates
		self.occupied_coords = set()
		self.unoccupied_coords = set()
		self.direction_mapping = {0: 'U', 1: 'L', 2: 'R', 3: 'D'}
		# dictionary mapping cells to direction of nearest occupied cells
		self.directions = dict()

		# dimensions and sets of medium and large cells
		self.med_dim = med
		self.large_dim = large
		self.small_squares = set()
		#map central coordinate of square to set of neighbors
		self.medium_squares = dict()
		self.large_squares = dict()

		#map dimensions
		self.x_max = None
		self.y_max = None

	def get_perp_neighbor_coords(self,coord):
		'''Returns perpendicular neighbors of an 
		(x, y) coordinate.'''
		x = coord[0]
		y = coord[1]

		#U, L, R, D
		neighbors = [(x-1, y),
					(x, y-1),(x, y+1),
					(x+1, y)]

		return neighbors

	def build_consolidated_map(self, map):

		self.x_max = map.shape[0]
		self.y_max = map.shape[1]

		#find occupied coordinates
		for x in range(self.x_max):
			for y in range(self.y_max):
				if map[x,y] == 1:
					self.occupied_coords.add((x, y))

		#create single square nodes
		for occupied_coord in self.occupied_coords:
			neighbors = self.get_perp_neighbor_coords(occupied_coord)
			for direction, coord in enumerate(neighbors):
				d = self.direction_mapping[direction]
				self.build_small_square(coord, d)
			
		

	def is_clear(self, coord, direction, size=1):

		if size != 1:
			#set up approrpriate parameters
			x = coord[0]
			y = coord[1]

			if direction == 'U' or direction == 'D':
				row_offset = 1
				column_offset = 0
				if direction == 'U': direction_offset = -1
				else: direction_offset = 1

			elif direction == 'L' or direction == 'R':
				row_offset = 0
				column_offset = 1
				if direction == 'L': direction_offset = -1
				else: direction_offset = 1

			coords_to_check = set()

			#bounds of possible new square
			x_bounds = (row_offset*direction_offset, (size+row_offset)*direction_offset, direction_offset)
			y_bounds = (column_offset*direction_offset, (size+column_offset)*direction_offset, direction_offset)

			for deltax in range(x_bounds[0], x_bounds[1], x_bounds[2]):
				for deltay in range(y_bounds[0], y_bounds[1], y_bounds[2]):
					coords_to_check.add((x+deltax, y+deltay))

		else:
			coords_to_check = {coord}

		for coord in coords_to_check:
			if coord in self.occupied_coords: return (False, None)
			if coord in self.unoccupied_coords: return (False, None)

		return (True, coords_to_check)

	def build_small_square(self, coord, direction):
		if self.is_clear(coord, direction)[0]:
			self.add_node(coord)
			self.unoccupied_coords.add(coord)
			self.small_squares.add(coord)
			self.directions[coord] = direction

	def build_medium_square(self, coords, direction):
		#find correct corner coordinate
		#check if surrounding area is clear
		pass

	def build_large_square(self, coord, direction):
		pass
	
