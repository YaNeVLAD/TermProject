#include "PathFinder.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <cmath>

void IntervalGraph::Build(const path_finder::VoronoiDiagram& vd, double spacing)
{
	obstacles = vd.GetObstacles();
	nodes.clear();
	edges.clear();

	for (const auto& edge : vd.GetEdges())
	{
		AddNodesAlongEdge(edge, spacing);
	}

	// Соединяем узлы на каждом ребре
	size_t node_count = nodes.size();
	for (size_t i = 0; i < node_count;)
	{
		size_t j = i + 1;
		while (j < node_count && nodes[j].edge == nodes[i].edge)
		{
			j++;
		}
		ConnectAdjacentNodesOnEdge(i, j - 1);
		i = j;
	}
}

void IntervalGraph::AddNodesAlongEdge(const path_finder::VoronoiDiagram::Edge& edge, double spacing)
{
	path_finder::Point start = edge.Start;
	path_finder::Point end = edge.End;
	path_finder::Point direction = (end - start).Normalize();
	double length = (end - start).Length();

	nodes.push_back({ start, &edge });
	for (double d = spacing; d < length; d += spacing)
	{
		path_finder::Point p = start + direction * d;
		nodes.push_back({ p, &edge });
	}
	nodes.push_back({ end, &edge });
}

void IntervalGraph::ConnectAdjacentNodesOnEdge(size_t start_idx, size_t end_idx)
{
	for (size_t i = start_idx; i < end_idx; ++i)
	{
		const auto& a = nodes[i];
		const auto& b = nodes[i + 1];
		double weight = (b.position - a.position).Length();
		edges.push_back({ i, i + 1, weight });
		edges.push_back({ i + 1, i, weight }); // Для неориентированного графа
	}
}

std::vector<path_finder::Point> IntervalGraph::FindPath(const path_finder::Point& start, const path_finder::Point& goal)
{
	// Создаём временные узлы для старта и цели
	GraphNode start_node{ start, nullptr };
	GraphNode goal_node{ goal, nullptr };
	nodes.push_back(start_node);
	nodes.push_back(goal_node);
	size_t start_idx = nodes.size() - 2;
	size_t goal_idx = nodes.size() - 1;

	// Создаём граф Boost
	using Graph = boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS,
		boost::no_property, boost::property<boost::edge_weight_t, double>>;
	using Vertex = boost::graph_traits<Graph>::vertex_descriptor;

	Graph g;
	for (const auto& edge : edges)
	{
		boost::add_edge(edge.from, edge.to, edge.weight, g);
	}

	// Соединяем старт и цель с ближайшими узлами
	for (size_t i = 0; i < nodes.size() - 2; ++i)
	{
		if (IsStraightPathValid(start, nodes[i].position))
		{
			double weight = (nodes[i].position - start).Length();
			boost::add_edge(start_idx, i, weight, g);
		}
		if (IsStraightPathValid(goal, nodes[i].position))
		{
			double weight = (nodes[i].position - goal).Length();
			boost::add_edge(goal_idx, i, weight, g);
		}
	}

	// Запускаем Дейкстру
	std::vector<Vertex> predecessors(boost::num_vertices(g));
	std::vector<double> distances(boost::num_vertices(g));

	boost::dijkstra_shortest_paths(g, start_idx,
		boost::predecessor_map(boost::make_iterator_property_map(predecessors.begin(), boost::get(boost::vertex_index, g)))
			.distance_map(boost::make_iterator_property_map(distances.begin(), boost::get(boost::vertex_index, g))));

	// Восстанавливаем путь
	if (distances[goal_idx] == std::numeric_limits<double>::max())
	{
		return {};
	}

	std::vector<path_finder::Point> path;
	for (Vertex v = goal_idx; v != start_idx; v = predecessors[v])
	{
		path.push_back(nodes[v].position);
	}
	path.push_back(start);
	std::reverse(path.begin(), path.end());

	return path;
}

bool IntervalGraph::IsStraightPathValid(const path_finder::Point& a, const path_finder::Point& b) const
{
	for (const auto& obstacle : obstacles)
	{
		if (obstacle.Intersects(a, b))
		{
			return false;
		}
	}
	return true;
}
