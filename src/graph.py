class Node(object):
	def __init__(self,x,y, val):
		self.position = (x,y)
		self.value = val

class Dimensioned_Node(Node):
	def __init__(x, y, val, dim):
		Node.__init__(x, y, val)
		self.dim = dim


class Edge(object):
	def __init__(self, start, end, cost=1):
		self.start = start
		self.end = end
		self.cost = cost


class Graph(object):
	def __init__(self):
		self.nodes = set()
		self.edges = {} # dict of node mapped to edges

	def add_node(self, node):
		self.nodes.add(node)
		self.edges[node] = []

	def add_edge(self, edge):
		self.edges[edge.start].append(edge)

	def __str__(self):
		p = dict()
		for node in self.nodes:
			p[node.position] = []
			for edge in self.edges[node]:
				p[node.position].append(edge.end.position)
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
				pos = Node(x, y, map[x, y])
				self.add_node(pos)
				for coord in self.get_neighbor_coords(pos):
					if 0 <= coord[0] <= x_max and 0 <= coord[1] <= y_max and map[x,y] != 1:
						ed = Edge(pos, Node(coord[0], coord[1],map[x, y]))
						self.add_edge(ed)

	def build_consolidated_map(self, map):

		def medium_square(coor):
			pass

		def large_square(coor):
			pass

			
		x_max = map.shape[0]
		y_max = map.shape[1]

	def heuristic(self, node1, node2):
		point1 = node1.position
		point2 = node2.position
		return ((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)**(0.5)
