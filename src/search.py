# Using A* algorithm searches graph made in graph_builder.py to find
# shortest path from starting position to ending position

start = "starting node in graph"
goal = "ending node in graph"
graph = "whatever data structure we are using for graph"


def a_star(graph, start, goal):
	frontier = []
	frontier.append((start,0)) # frontier consists of nodes 
	came_from = {}
	current_cost = {}
	came_from[start] = None
	current_cost[start] = 0

	while len(frontier)!=0:
		current = frontier.pop(0)
		if current == goal:
			break

		neighbors = graph.edges[current] # list of edges
		for edge in neighbors:
			n = edge.end
			new_cost = current_cost[current] + edge.cost
			if n not in current_cost or new_cost<current_cost[n]:
				current_cost[n] = new_cost
				priority = new_cost + graph.heuristic(goal, n)
				frontier = queue_insert(frontier, n, priority)

				came_from[n] = current


def queue_insert(queue, node, weight):
	if len(queue) == 0:
		return [node]
	else:
		for i in range(len(queue)):
			if queue[i][1]>weight:
				return queue[:i] + weight + queue[i:]