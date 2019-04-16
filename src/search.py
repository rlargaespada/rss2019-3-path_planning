# Using A* algorithm searches graph made in graph_builder.py to find
# shortest path from starting position to ending position


def a_star(graph, start, goal):
	''' Algorithm ends with a dictionary mapping nodes to their parent nodes'''
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

		for n in graph.neighbors[current]:
			new_cost = current_cost[current] + graph.cost(current,n)
			if n not in current_cost or new_cost<current_cost[n]:
				current_cost[n] = new_cost
				priority = new_cost + graph.heuristic(n)
				frontier = queue_insert(frontier, n, priority)
				came_from[n] = current

	return process_astar(came_from, start, goal)


def queue_insert(queue, node, weight):
	if len(queue) == 0:
		return [node]
	else:
		for i in range(len(queue)):
			if queue[i][1]>weight:
				return queue[:i] + weight + queue[i:]

def process_astar(came_from, start, goal):
	points = []
	node = goal
	while node!=start:
		points.append(node)
		node = came_from[node]
	points.append(start)
	return points[::-1]