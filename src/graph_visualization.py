# attempting to visualize the graph created from the map of the stata basement using networkx
# put into graph_builder when working

import matplotlib.pyplot as plt

import networkx as nx


def visualize_graph(stata_graph):
	g = nx.Graph()
	id_map = {}
	node_positions = {}
	count = 0
	for in stata_graph.nodes:
		g.node[str(count)]={"X": n[0], "Y": n[1]}
		id_map[n]=str(count)
		node_positions[str(count)]=n
		count+=1
	for start, ends in stata_map.neighbors:
		for end in ends:
			g.add_edge(id_map[start], id_map[end], {'distance':stata_graph.cost(start, end),'color': 'red'})
	plt.figure(figsize=(8, 6)) # needs to be modified
	nx.draw(g, pos=node_positions, edge_color=edge_colors, node_size=10, node_color='black')
	plt.show()