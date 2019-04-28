class Node(object):
	def __init__(self,x,y):
		self.position = (x,y)

class Edge(object):
	def __init__(self, start, end, cost):
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

	def heuristic(self, node1, node2):
		point1 = node1.position
		point2 = node2.position
		return ((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)**(0.5)
