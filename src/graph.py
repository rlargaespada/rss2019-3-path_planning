import json
import numpy as np
#import decimal

class Graph(object):
	def __init__(self, start, goal):
		self.nodes = set()
		self.neighbors = {} # dict of node mapped to edges
		self.start = start
		self.goal = goal
		self.x_max = None
		self.y_max = None
		self.resolution = None
		self.origin = tuple()

	def cost(self, p1, p2):
		return ((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)**(0.5)

	def add_node(self, node):
		if node in self.nodes:
			return
		self.nodes.add(node)
		self.neighbors[node] = set()

	def add_edge(self, node1, node2):
		self.neighbors[node1].add(node2)

	def __str__(self):
		for node in self.neighbors:
			print(node, ' : ', self.neighbors[node])
		return str(len(self.nodes))

	def get_neighbor_coords(self,coord):
		x = coord[0]
		y = coord[1]

		neighbors = {(x-1, y-1), (x-1, y), (x-1, y+1),
					(x, y-1),(x, y+1),
					(x+1, y-1),(x+1, y),(x+1, y+1)}

		return neighbors

	def occ_to_real_world(self, coord):
		x = round(-coord[0]*self.resolution + self.origin[0], 1)
		y = round(-coord[1]*self.resolution + self.origin[1], 1)
		return (x,y)

	def build_map(self, map, name, resolution, origin):
		#fn = str(name)+'.json'
		# try: 
		# 	with open(fn, 'r') as fp:
		# 		data = json.load(fp)
		# 	self.neighbors = data
		# 	self.nodes = set(self.neighbors.keys())
		# 	return
		# except:
		# 	pass
		self.resolution = resolution
		self.origin = origin
		self.x_max = map.shape[0]-1
		self.y_max = map.shape[1]-1

		for x in range(0, self.x_max+1, 1):
			for y in range(0, self.y_max+1, 1):
				if map[x,y] == 0:
					pos = (x,y)
					rwpose = self.occ_to_real_world(pos)
					self.add_node(rwpose)
					for coord in self.get_neighbor_coords(pos):
						x_prime = coord[0]
						y_prime = coord[1]
						if 0 <= x_prime <= self.x_max and 0 <= y_prime <= self.y_max:
							if map[x_prime, y_prime] == 0:
								rwcoord = self.occ_to_real_world(coord)
								self.add_node(rwcoord)
								self.add_edge(rwpose, rwcoord)

		# with open(fn, 'w') as fp:
		# 	json.dump(self.neighbors, fp)
		# return

	def heuristic(self, node1):
		point1 = node1
		point2 = self.goal
		return ((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)**(0.5)

class Lookahead_Graph(Graph):
	def __init__(self, start, goal, lookahead):
		Graph.__init__(self, start, goal)
		self.lookahead = lookahead #decimal.Decimal(str(lookahead))
		self.x_min = None
		self.y_min = None
		self.map = None

	def occ_to_real_world(self, coord):
		x = round(-coord[0]*self.resolution + self.origin[0], 2)
		y = round(-coord[1]*self.resolution + self.origin[1], 2)
		return (x,y)

	def real_world_to_occ(self, coord):
		x = round((-coord[0]+self.origin[0])/self.resolution, 1)
		y = round((-coord[1]+self.origin[1])/self.resolution, 1)
		return (x,y)

	def test_setup(self, origin, resolution, map):
		self.map = map
		self.resolution = resolution #decimal.Decimal(str(resolution))
		self.origin = origin #[decimal.Decimal(str(dim)) for dim in origin]
		self.x_max, self.y_max = self.occ_to_real_world((0,0))
		# self.x_max = decimal.Decimal(str(x_max))
		# self.y_max = decimal.Decimal(str(y_max))
		self.x_min, self.y_min = self.occ_to_real_world((self.map.shape[0]-1,self.map.shape[1]-1))
		# self.x_min = decimal.Decimal(str(x_min))
		# self.y_min = decimal.Decimal(str(y_min))

	def insert_node(self, node): #adding additional nodes after the graph is built
		self.add_node(node)
		for n in self.nodes:
			if n != node and self.get_dist(node, n) < self.lookahead/2:
				self.add_edge(node, n)
				self.add_edge(n, node)

	def get_lookahead_neighbors(self, coord, map):
		# x = coord[0] #rw
		# y = coord[1] #rw

		coord_oc = self.real_world_to_occ(coord)
		x_oc = int(coord_oc[0])
		y_oc = int(coord_oc[1])
		x_max = map.shape[0]-1
		y_max = map.shape[1]-1
		lookahead_neighbors = set()
		lookahead = int(round(self.lookahead/self.resolution, 2))

		#lower right diagonal
		for deltaxy in range(1, lookahead, 1):
			neighbor = (x_oc+deltaxy, y_oc+deltaxy)
			if not (0 <= neighbor[0] <= x_max) or not (0 <= neighbor[1] <= y_max):
				n = ((x_oc+deltaxy-1, y_oc+deltaxy-1))
				if n != coord_oc:
					lookahead_neighbors.add(self.occ_to_real_world(n))
				break
			if map[int(neighbor[0]), int(neighbor[1])] != 0:
				n = ((x_oc+deltaxy-1, y_oc+deltaxy-1))
				if n!= coord_oc:
					lookahead_neighbors.add(self.occ_to_real_world(n))
				break
			elif deltaxy == lookahead-1:
				n = ((x_oc+deltaxy, y_oc+deltaxy))
				lookahead_neighbors.add(self.occ_to_real_world(n))

		#upper right diagonal
		for deltaxy in range(1, lookahead, 1):
			neighbor = (x_oc-deltaxy, y_oc+deltaxy)
			if not (0 <= neighbor[0] <= x_max) or not (0 <= neighbor[1] <= y_max):
				n = ((x_oc-deltaxy+1, y_oc+deltaxy-1))
				if n != coord_oc:
					lookahead_neighbors.add(self.occ_to_real_world(n))
				break
			if map[int(neighbor[0]), int(neighbor[1])] != 0:
				n = ((x_oc-deltaxy+1, y_oc+deltaxy-1))
				if n!= coord_oc:
					lookahead_neighbors.add(self.occ_to_real_world(n))
				break
			elif deltaxy == lookahead-1:
				n = ((x_oc-deltaxy, y_oc+deltaxy))
				lookahead_neighbors.add(self.occ_to_real_world(n))

		#lower left diagonal
		for deltaxy in range(1, lookahead, 1):
			neighbor = (x_oc+deltaxy, y_oc-deltaxy)
			if not (0 <= neighbor[0] <= x_max) or not (0 <= neighbor[1] <= y_max):
				n = ((x_oc+deltaxy-1, y_oc-deltaxy+1))
				if n != coord_oc:
					lookahead_neighbors.add(self.occ_to_real_world(n))
				break
			if map[int(neighbor[0]), int(neighbor[1])] != 0:
				n = ((x_oc+deltaxy-1, y_oc-deltaxy+1))
				if n!= coord_oc:
					lookahead_neighbors.add(self.occ_to_real_world(n))
				break
			elif deltaxy == lookahead-1:
				n = ((x_oc+deltaxy, y_oc-deltaxy))
				lookahead_neighbors.add(self.occ_to_real_world(n))

		#upper left diagonal
		for deltaxy in range(1, lookahead, 1):
			neighbor = (x_oc-deltaxy, y_oc-deltaxy)
			if not (0 <= neighbor[0] <= x_max) or not (0 <= neighbor[1] <= y_max):
				n = ((x_oc-deltaxy+1, y_oc-deltaxy+1))
				if n != coord_oc:
					lookahead_neighbors.add(self.occ_to_real_world(n))
				break
			if map[int(neighbor[0]), int(neighbor[1])] != 0:
				n = ((x_oc-deltaxy+1, y_oc-deltaxy+1))
				if n!= coord_oc:
					lookahead_neighbors.add(self.occ_to_real_world(n))
				break
			elif deltaxy == lookahead-1:
				n = ((x_oc-deltaxy, y_oc-deltaxy))
				lookahead_neighbors.add(self.occ_to_real_world(n))

		#upper/lower neighbors
		x_bound = int(x_oc-lookahead+1)
		upper_col = map[:x_oc+1, y_oc] if x_bound < 0 else map[x_bound:x_oc+1, y_oc]

		for deltax in range(-2, -len(upper_col)-1, -1):
			if upper_col[deltax] != 0:
				if deltax != -2:
					lookahead_neighbors.add(self.occ_to_real_world((x_oc+deltax+2, y_oc)))
				break
			elif deltax == -len(upper_col):
				lookahead_neighbors.add(self.occ_to_real_world((x_oc+deltax+1, y_oc)))

		lower_col = map[x_oc:x_oc+lookahead, y_oc] #contains x in lower_col[0]
		for deltax in range(1, len(lower_col), 1):
			if lower_col[deltax] != 0:
				if deltax != 1:
					lookahead_neighbors.add(self.occ_to_real_world((x_oc+deltax-1, y_oc)))
				break
			elif deltax == len(lower_col)-1:
				lookahead_neighbors.add(self.occ_to_real_world((x_oc+deltax, y_oc)))

		#left/right neighbors
		y_bound = y_oc-lookahead+1
		left_row = map[x_oc, :y_oc+1] if y_bound < 0 else map[x_oc, y_bound:y_oc+1]
		right_row = map[x_oc, y_oc:lookahead+y_oc]

		for deltay in range(-2, -len(left_row)-1, -1):
			if left_row[deltay] != 0:
				if deltay != -2:
					lookahead_neighbors.add(self.occ_to_real_world((x_oc, y_oc+deltay+2)))
				break
			elif deltay == -len(left_row):
				lookahead_neighbors.add(self.occ_to_real_world((x_oc, y_oc+deltay+1)))
		
		for deltay in range(1, len(right_row), 1):
			if right_row[deltay] != 0:
				if deltay != 1:
					lookahead_neighbors.add(self.occ_to_real_world((x_oc, y_oc+deltay-1)))
				break
			elif deltay == len(right_row)-1:
				lookahead_neighbors.add(self.occ_to_real_world((x_oc, y_oc+deltay)))

		return lookahead_neighbors

	def nodes_in_range(self, node, current):
		for n in self.nodes:
			if n != current and self.get_dist(node, n) < self.lookahead/2: return n
		return node

	def get_dist(self, pos1, pos2):
		return ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)**.5	

	def build_map(self, map, name, resolution, origin):
		self.resolution = resolution
		self.origin = origin
		self.x_max, self.y_max = self.occ_to_real_world((0,0))
		self.x_min, self.y_min = self.occ_to_real_world((map.shape[0]-1,map.shape[1]-1))

		# self.start = self.occ_to_real_world(self.start)
		# self.goal = self.occ_to_real_world(self.goal)
		
		# fn = str(name)+'.json'
		# try: 
		# 	with open(fn, 'r') as fp:
		# 		data = json.load(fp)
		# 	self.neighbors = data
		# 	self.nodes = set(self.neighbors.keys())
		# 	return
		# except:
		# 	pass
		self.add_node(self.start) #rw
		self.add_node(self.goal) #rw
		neighbor_queue = [self.start] #rw

		while len(neighbor_queue) != 0:
			current = neighbor_queue.pop() #rw
			self.add_node(current) #rw
			neighbors = self.get_lookahead_neighbors(current, map) #rw
			for neighbor in neighbors:
				n = self.nodes_in_range(neighbor, current) #rw
				self.add_edge(current, n) #rw, rw
				if n not in self.nodes: neighbor_queue.append(n) #rw

		self.insert_node(self.goal)
				
		# with open(fn, 'w') as fp:
		# 	json.dump(self.neighbors, fp)
	
		
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

	def build_map(self, map):

		self.x_max = map.shape[0]-1
		self.y_max = map.shape[1]-1

		#find occupied coordinates
		for x in range(self.x_max+1):
			for y in range(self.y_max+1):
				if map[x,y] != 0:
					self.occupied_coords.add((x, y))

		#create single square nodes
		for occupied_coord in self.occupied_coords:
			neighbors = self.get_perp_neighbor_coords(occupied_coord)
			for direction, coord in enumerate(neighbors):
				d = self.direction_mapping[direction]
				self.build_small_square(coord, d)

		#create medium squares
		for square in self.small_squares:
			for d in self.direction_mapping.values():
				self.build_medium_square(square, d)

			
		#create large squares
		

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
			if not (0 <= coord[0] <= self.x_max) or not (0 <= coord[1] <= self.y_max): return (False, coords_to_check)
			if coord in self.occupied_coords: return (False, coords_to_check)
			if coord in self.unoccupied_coords: return (False, coords_to_check)

		return (True, coords_to_check)

	def build_small_square(self, coord, direction):
		if self.is_clear(coord, direction)[0]:
			self.add_node(coord)
			self.unoccupied_coords.add(coord)
			self.small_squares.add(coord)
			self.directions[coord] = direction
			for d, neighbor in enumerate(self.get_perp_neighbor_coords(coord)):
				#fix for when a small square is made from possible medium squares
				if self.direction_mapping[d] != direction:
					self.add_edge(coord, neighbor)

	def build_medium_square(self, coords, direction):
		#find correct corner coordinate
		#check if surrounding area is clear
		clear, area = self.is_clear(coords, direction)
		if clear:
			#create medium square in using the 9 squares in area
			pass
		else:
			#create small squares using all working squares in area
			for square in area:
				self.build_small_square(square, direction)

	def build_large_square(self, coord, direction):
		pass
	

#ul
# for deltaxy in np.linspace(step, self.lookahead, self.lookahead/step):
# 	deltaxy = round(deltaxy, 2)
# 	neighbor = (round(x+deltaxy, 2), round(y+deltaxy, 2))
# 	if not (self.x_min <= neighbor[0] <= self.x_max) or not (self.y_min <= neighbor[1] <= self.y_max):
# 		n = ((round(x+deltaxy-step,2), round(y+deltaxy-step,2)))
# 		if n != coord:
# 			lookahead_neighbors.append(n)
# 		break
# 	map_test = (self.real_world_to_occ(neighbor))
# 	if map[int(map_test[0]), int(map_test[1])] != 0:
# 		n = ((round(x+deltaxy-step,2), round(y+deltaxy-step,2)))
# 		if n!= coord:
# 			lookahead_neighbors.append(n)
# 		break
# 	elif deltaxy == round(self.lookahead-step,2):
# 		n = ((round(x+deltaxy,2), round(y+deltaxy,2)))
# 		lookahead_neighbors.append(n)

#ll
# for deltaxy in np.linspace(step, self.lookahead, self.lookahead/step):
# 	deltaxy = round(deltaxy, 2)
# 	neighbor = (round(x-deltaxy,2), round(y+deltaxy,2))
# 	if not (self.x_min <= neighbor[0] <= self.x_max) or not (self.y_min <= neighbor[1] <= self.y_max):
# 		n = ((round(x-deltaxy+step,2), round(y+deltaxy-step,2)))
# 		if n != coord:
# 			lookahead_neighbors.append(n)
# 		break
# 	map_test = (self.real_world_to_occ(neighbor))
# 	if map[int(map_test[0]), int(map_test[1])] != 0:
# 		n = ((round(x-deltaxy+step,2), round(y+deltaxy-step,2)))
# 		if n!= coord:
# 			lookahead_neighbors.append(n)
# 		break
# 	elif deltaxy == round(self.lookahead-step,2):
# 		n = ((round(x-deltaxy,2), round(y+deltaxy,2)))
# 		lookahead_neighbors.append(n)

#ur
# for deltaxy in np.linspace(step, self.lookahead-step, self.lookahead/step):
# 	deltaxy = round(deltaxy, 2)
# 	neighbor = (round(x+deltaxy, 2), round(y-deltaxy, 2))
# 	if not (self.x_min <= neighbor[0] <= self.x_max) or not (self.y_min <= neighbor[1] <= self.y_max):
# 		n = ((round(x+deltaxy-step,2), round(y-deltaxy+step,2)))
# 		if n != coord:
# 			lookahead_neighbors.append(n)
# 		break
# 	map_test = (self.real_world_to_occ(neighbor))
# 	if map[int(map_test[0]), int(map_test[1])] != 0:
# 		n = ((round(x+deltaxy-step,2), round(y-deltaxy+step,2)))
# 		if n!= coord:
# 			lookahead_neighbors.append(n)
# 		break
# 	elif deltaxy == round(self.lookahead-step,2):
# 		n = ((round(x+deltaxy,2), round(y-deltaxy,2)))
# 		lookahead_neighbors.append(n)

#lr
# for deltaxy in np.linspace(step, self.lookahead-step, self.lookahead/step):
# 	deltaxy = round(deltaxy, 2)
# 	neighbor = (round(x-deltaxy, 2), round(y-deltaxy, 2))
# 	if not (self.x_min <= neighbor[0] <= self.x_max) or not (self.y_min <= neighbor[1] <= self.y_max):
# 		#out of bounds, add boundary
# 		n = ((round(x-deltaxy+step,2), round(y-deltaxy+step,2)))
# 		if n != coord:
# 			lookahead_neighbors.append(n)
# 		break
# 	map_test = (self.real_world_to_occ(neighbor))
# 	if map[int(map_test[0]), int(map_test[1])] != 0:
# 		#wall, add boundary
# 		n = ((round(x-deltaxy+step,2), round(y-deltaxy+step, 2)))
# 		if n!= coord:
# 			lookahead_neighbors.append(n)
# 		break
# 	elif deltaxy == round(self.lookahead-step,2):
# 		#lookahead limit, add neighbor here
# 		n = ((round(x-deltaxy,2), round(y-deltaxy,2)))
# 		lookahead_neighbors.append(n)

#lr decimal
# deltaxy = decimal.Decimal(0)
# while deltaxy < self.lookahead:
# 	deltaxy += step
# 	print('deltaxy: ', deltaxy)
# 	neighbor = (x-deltaxy, y-deltaxy)
# 	print('neighbor: ', neighbor)
# 	if not (self.x_min <= neighbor[0] <= self.x_max) or not (self.y_min <= neighbor[1] <= self.y_max):
# 		#out of bounds, add boundary
# 		n = ((x-deltaxy+step, y-deltaxy+step))
# 		if n != coord:
# 			lookahead_neighbors.add(n)
# 		break
# 	map_test = self.real_world_to_occ(neighbor)
# 	print('map_test: ', map_test)
# 	if map[int(map_test[0]), int(map_test[1])] != 0:
# 		#wall, add boundary
# 		n = (x-deltaxy+step, y-deltaxy+step)
# 		print('n: ', n)
# 		if n!= coord:
# 			lookahead_neighbors.add(n)
# 		break
# 	elif deltaxy == self.lookahead-step:
# 		print('here')
# 		#lookahead limit, add neighbor here
# 		n = (x-deltaxy, y-deltaxy)
# 		lookahead_neighbors.add(n)