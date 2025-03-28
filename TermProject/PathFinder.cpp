#include "PathFinder.h"

#include <cmath>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>

namespace path_finder
{

std::vector<Point> IntervalGraph::FindPath(const Point& start, const Point& goal)
{
	using namespace boost;

	std::vector<Point> path;

	GraphNode start_node{
		start,
		GetClearance(start),
		SIZE_MAX,
		nullptr,
		FindNearestObstacle(start), // Новый метод
		true
	};

	GraphNode goal_node{
		goal,
		GetClearance(goal),
		SIZE_MAX,
		nullptr,
		FindNearestObstacle(goal),
		true
	};

	// Добавляем временные узлы в граф
	size_t start_idx = nodes.size();
	size_t goal_idx = nodes.size() + 1;
	nodes.push_back(start_node);
	nodes.push_back(goal_node);

	// 1. Создаём граф для Boost
	typedef boost::adjacency_list<listS, vecS, directedS,
		no_property, property<edge_weight_t, double>>
		Graph;
	typedef graph_traits<Graph>::vertex_descriptor Vertex;

	Graph g;
	property_map<Graph, edge_weight_t>::type weight_map = get(edge_weight, g);

	// 2. Добавляем ВСЕ узлы + временные вершины
	std::vector<Vertex> vertices(nodes.size() + 2); // +2 для start/goal
	for (size_t i = 0; i < nodes.size() + 2; ++i)
	{
		vertices[i] = add_vertex(g);
	}

	// 3. Добавляем оригинальные рёбра
	for (const Edge& e : edges)
	{
		auto edge_result = add_edge(vertices[e.from], vertices[e.to], g);
		weight_map[edge_result.first] = e.weight;
	}

	// Находим ближайшие узлы в графе
	size_t nearest_to_start = FindNearestNode(start);
	size_t nearest_to_goal = FindNearestNode(goal);

	// Добавляем рёбра start -> nearest_to_start
	auto [start_path, start_weight] = CalculateConnection(start_node, nodes[nearest_to_start]);
	add_edge(vertices[start_idx], vertices[nearest_to_start], start_weight, g);

	// Добавляем рёбра nearest_to_goal -> goal
	auto [goal_path, goal_weight] = CalculateConnection(nodes[nearest_to_goal], goal_node);
	add_edge(vertices[nearest_to_goal], vertices[goal_idx], goal_weight, g);

	// 5. Запускаем Дейкстру
	std::vector<Vertex> predecessors(num_vertices(g));
	std::vector<double> distances(num_vertices(g));

	dijkstra_shortest_paths(g, vertices[start_idx],
		predecessor_map(make_iterator_property_map(predecessors.begin(), get(vertex_index, g)))
			.distance_map(make_iterator_property_map(distances.begin(), get(vertex_index, g))));

	// 6. Восстанавливаем путь (исключая временные вершины)
	if (distances[vertices[goal_idx]] == std::numeric_limits<double>::max())
	{
		return path;
	}

	std::vector<Vertex> vertex_path;
	for (Vertex v = vertices[goal_idx];; v = predecessors[v])
	{
		size_t idx = get(vertex_index, g, v);
		if (idx < nodes.size())
		{ // Исключаем start_idx и goal_idx
			path.push_back(nodes[idx].position);
		}
		if (v == predecessors[v])
			break;
	}
	std::reverse(path.begin(), path.end());

	// Добавляем оригинальные start/goal для точности
	path.insert(path.begin(), start);
	path.push_back(goal);

	return path;
}

const Obstacle* IntervalGraph::FindNearestObstacle(const Point& p) const
{
	const Obstacle* nearest = nullptr;
	double min_dist = std::numeric_limits<double>::max();

	for (const auto& obstacle : m_Obstacles)
	{
		double dist = (p - obstacle.ClosestPoint(p)).Length();
		if (dist < min_dist)
		{
			min_dist = dist;
			nearest = &obstacle;
		}
	}
	return nearest;
}

void IntervalGraph::AddNode(const GraphNode& node)
{
	nodes.push_back(node);
}
void IntervalGraph::ConnectIntervals(const VoronoiDiargram& vd)
{
	auto adjacent = vd.GetAdjacentIntervals();

	for (const auto& [i, j] : adjacent)
	{
		// Проверяем валидность индексов
		if (i >= nodes.size() || j >= nodes.size())
			continue;

		// Получаем связанные узлы
		const auto& node_i = nodes[i];
		const auto& node_j = nodes[j];

		// Вычисляем соединение только между конечными точками
		auto [path, weight] = CalculateConnection(node_i, node_j);

		if (IsPathClear(path))
		{
			edges.push_back({ i, j, weight });
			edges.push_back({ j, i, weight }); // Для неориентированного графа
		}
	}
}

std::pair<std::vector<Point>, double> IntervalGraph::CalculateConnection(const GraphNode& a, const GraphNode& b)
{
	if (a.is_temporary || b.is_temporary)
	{
		return CalculateLinearSegment(a, b);
	}

	if (!a.edge || !b.edge)
	{
		return { {}, std::numeric_limits<double>::max() };
	}

	if (a.edge->LeftFeature == VoronoiDiargram::Edge::FeatureType::Vertex)
	{
		return CalculateSpiralArc(a, b);
	}
	else
	{
		return CalculateCircularArc(a, b);
	}
}

std::pair<std::vector<Point>, double>
IntervalGraph::CalculateLinearSegment(const GraphNode& a, const GraphNode& b)
{
	// Проверяем коллизии для всего отрезка
	if (!IsPathClear({ a.position, b.position }))
	{
		return { {}, std::numeric_limits<double>::max() };
	}

	// Расчёт веса на основе клиренса
	double length = (b.position - a.position).Length();
	double c_avg = (a.clearance + b.clearance) / 2.0;
	double weight = length * pow(1.0 / c_avg, m_delta);

	return { { a.position, b.position }, weight };
}

std::pair<std::vector<Point>, double> IntervalGraph::CalculateSpiralArc(const GraphNode& a, const GraphNode& b)
{
	double theta1 = atan2(a.position.Y, a.position.X);
	double theta2 = atan2(b.position.Y, b.position.X);
	double r1 = a.position.Length();
	double r2 = b.position.Length();

	// Вычисляем параметры спирали
	double b_param = (log(r2) - log(r1)) / (theta2 - theta1);
	double a_param = r1 / exp(b_param * theta1);

	// Дискретизация спирали и расчёт веса
	std::vector<Point> points;
	double total_weight = 0.0;
	Point prev_point = a.position;

	for (double theta = theta1; theta <= theta2; theta += 0.1)
	{
		double r = a_param * exp(b_param * theta);
		Point current(r * cos(theta), r * sin(theta));
		points.push_back(current);

		double segment_length = (current - prev_point).Length();
		double clearance = (GetClearance(prev_point) + GetClearance(current)) / 2;
		total_weight += segment_length * pow(1.0 / clearance, m_delta);

		prev_point = current;
	}

	return { points, total_weight };
}

std::pair<std::vector<Point>, double> IntervalGraph::CalculateCircularArc(const GraphNode& a, const GraphNode& b)
{
	// Нормализованное направление ребра препятствия
	Point edge_dir = (a.edge->End - a.edge->Start).Normalize();
	Point normal(-edge_dir.Y, edge_dir.X); // Нормаль к ребру

	// Параметры окружности
	double radius = a.clearance;
	Point center = a.edge->Start + normal * radius;

	std::vector<Point> points;
	double weight = 0.0;
	Point prev_point;

	// Параметризация дуги с шагом 5% от угла 90 градусов (п/2)
	for (double t = 0; t <= 1.0; t += 0.05)
	{
		Point current_point = center + (a.position - center) * cos(t * (3.14 / 2)) + normal * radius * sin(t * (3.14 / 2));

		points.push_back(current_point);

		// Расчёт веса начинаем со второго шага
		if (points.size() >= 2)
		{
			// 1. Длина сегмента между предыдущей и текущей точкой
			double segment_length = (current_point - prev_point).Length();

			// 2. Средний клиренс на сегменте
			double c_prev = GetClearance(prev_point);
			double c_current = GetClearance(current_point);
			double c_avg = (c_prev + c_current) / 2.0;

			// 3. Интегрируем: (1/c)^
			weight += segment_length * std::pow(1.0 / c_avg, m_delta);
		}

		prev_point = current_point;
	}

	return { points, weight };
}

size_t IntervalGraph::FindNearestNode(const Point& p) const
{
	size_t nearest_idx = 0;
	double min_dist = std::numeric_limits<double>::max();

	for (size_t i = 0; i < nodes.size(); ++i)
	{
		double dist = (nodes[i].position - p).Length();
		if (dist < min_dist)
		{
			min_dist = dist;
			nearest_idx = i;
		}
	}
	return nearest_idx;
}

bool IntervalGraph::IsPathClear(const std::vector<Point>& path) const
{
	if (path.empty())
	{
		return false;
	}

	for (const auto& obstacle : m_Obstacles)
	{
		if (obstacle.Intersects(path.front(), path.back()))
		{
			return false;
		}
	}
	return true;
}

void IntervalGraph::BuildAdjacencyList()
{
	adjacency_list.resize(nodes.size());
	for (size_t i = 0; i < edges.size(); ++i)
	{
		adjacency_list[edges[i].from].push_back(i);
		adjacency_list[edges[i].to].push_back(i);
	}
}

double IntervalGraph::GetClearance(const Point& p) const
{
	double min_dist = std::numeric_limits<double>::max();
	for (const auto& obstacle : m_Obstacles)
	{
		double dist = (p - obstacle.ClosestPoint(p)).Length();
		if (dist < min_dist)
		{
			min_dist = dist;
		}
	}
	return min_dist;
}

double IntervalGraph::CalculateSpiralWeight(const std::vector<Point>& path, double delta)
{
	double total = 0.0;
	for (size_t i = 1; i < path.size(); ++i)
	{
		double segment_length = (path[i] - path[i - 1]).Length();
		double clearance = (GetClearance(path[i]) + GetClearance(path[i - 1])) / 2;
		total += segment_length / std::pow(clearance, delta);
	}
	return total;
}

} // namespace path_finder
