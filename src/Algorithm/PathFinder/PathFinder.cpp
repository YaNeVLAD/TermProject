//
// Created by User on 15.04.2025.
//

#include "PathFinder.h"

#include "../Math/Math.hpp"
#include "../Structures/BoostCompat.h"

using namespace tp;
using namespace tp::shapes;

namespace
{
double CalculateEdgeCost(const Point& p1, const Point& p2,
	const std::vector<Polygon>& obstacles)
{
	using namespace tp::shapes;
	using namespace tp::math;
	float segment_length = Distance(p1, p2);
	if (segment_length < FLT_EPSILON)
	{
		return 0.0;
	}

	// Лямбда-функция для вычисления зазора в точке p
	auto get_clearance = [&](const Point& p) -> float {
		float min_dist_sq = std::numeric_limits<float>::max();
		if (obstacles.empty())
		{
			return std::sqrt(std::numeric_limits<float>::max());
		}

		for (const auto& polygon : obstacles)
		{
			if (polygon.empty())
				continue;
			for (size_t i = 0; i < polygon.size(); ++i)
			{
				const auto& v1 = polygon[i];
				const auto& v2 = polygon[(i + 1) % polygon.size()];
				float d_sq = DistanceToSegmentSquared(p, v1, v2);
				if (d_sq < min_dist_sq)
				{
					min_dist_sq = d_sq;
				}
			}
		}

		if (min_dist_sq < FLT_EPSILON * FLT_EPSILON)
			return FLT_EPSILON;
		return std::sqrt(min_dist_sq);
	};

	// Численное интегрирование методом левых прямоугольников
	int num_steps = 50; // Количество шагов для интегрирования. Можно сделать параметром.
	// Если num_steps = 0 или segment_length = 0, результат должен быть 0. segment_length уже проверен.

	double integral_sum = 0.0;
	Vec2D segment_vector = p2 - p1; // Вектор от p1 к p2

	// Длина каждого малого шага интегрирования
	float delta_s = segment_length / static_cast<float>(num_steps);

	for (int i = 0; i < num_steps; ++i) // Итерации от i = 0 до num_steps - 1
	{
		// t - параметр от 0 до (почти) 1, соответствующий началу i-го шага
		float t = static_cast<float>(i) / static_cast<float>(num_steps);
		Point current_step_start_point = p1 + segment_vector.Scale(t); // Точка x_i

		float clearance_at_step_start = get_clearance(current_step_start_point);

		if (clearance_at_step_start < FLT_EPSILON)
		{
			// Если зазор критически мал (например, точка внутри или на границе препятствия),
			// стоимость этого участка пути должна быть очень высокой.
			return std::numeric_limits<double>::max();
		}

		// Добавляем член суммы для метода прямоугольников: (1 / зазор)
		// Умножение на delta_s произойдет в конце для всей суммы.
		integral_sum += (1.0 / static_cast<double>(clearance_at_step_start));
	}

	// Итоговая стоимость: сумма (1/зазор_i) * длина_шага_ds
	double cost = integral_sum * static_cast<double>(delta_s);

	return cost;
}

struct GraphData
{
	boost_compat::Graph graph;
	std::map<Point, boost_compat::Vertex> point_to_vertex_map;
};

GraphData ConstructG1_Graph(
	const std::vector<Segment>& segments_tilde_V,
	const Point& start_node_s,
	const Point& goal_node_t,
	const std::vector<Polygon>& obstacles,
	const std::vector<Point>& N_s,
	const std::vector<Point>& N_t)
{
	using namespace boost_compat;

	std::vector<Point> graph_points_collector;
	graph_points_collector.push_back(start_node_s);
	graph_points_collector.push_back(goal_node_t);

	for (const auto& [p1, p2] : segments_tilde_V)
	{
		graph_points_collector.push_back(p1);
		graph_points_collector.push_back(p2);
	}
	for (const auto& p_ns : N_s)
	{
		graph_points_collector.push_back(p_ns);
	}
	for (const auto& p_nt : N_t)
	{
		graph_points_collector.push_back(p_nt);
	}

	std::sort(graph_points_collector.begin(), graph_points_collector.end());
	graph_points_collector.erase(
		std::ranges::unique(graph_points_collector).begin(),
		graph_points_collector.end());

	std::map<Point, Vertex> point_to_vertex_map_local;
	Graph G1_local(graph_points_collector.size());

	for (size_t i = 0; i < graph_points_collector.size(); ++i)
	{
		Vertex v_desc = vertex(i, G1_local);
		point_to_vertex_map_local[graph_points_collector[i]] = v_desc;
		G1_local[v_desc] = graph_points_collector[i];
	}

	Vertex s_vertex_local = point_to_vertex_map_local.at(start_node_s);
	Vertex t_vertex_local = point_to_vertex_map_local.at(goal_node_t);

	// 1. Добавляем ребра (w, w') для segments_tilde_V
	for (const auto& [p1, p2] : segments_tilde_V)
	{
		if (point_to_vertex_map_local.contains(p1) && point_to_vertex_map_local.contains(p2) && p1 != p2)
		{
			Vertex u = point_to_vertex_map_local.at(p1);
			Vertex v = point_to_vertex_map_local.at(p2);
			double cost = CalculateEdgeCost(p1, p2, obstacles);
			if (cost < std::numeric_limits<double>::max())
			{
				boost::add_edge(u, v, cost, G1_local);
			}
		}
	}

	// 2. Добавляем ребра (s, w) для w из N_s
	for (const auto& w_point : N_s)
	{
		if (point_to_vertex_map_local.count(w_point))
		{
			Vertex w_vertex = point_to_vertex_map_local.at(w_point);
			if (start_node_s != w_point)
			{
				double cost_sw = CalculateEdgeCost(start_node_s, w_point, obstacles);
				if (cost_sw < std::numeric_limits<double>::max())
				{
					boost::add_edge(s_vertex_local, w_vertex, cost_sw, G1_local);
				}
			}
		}
	}

	// 3. Добавляем ребра (w, t) для w из N_t
	for (const auto& w_point : N_t)
	{
		if (point_to_vertex_map_local.contains(w_point))
		{
			Vertex w_vertex = point_to_vertex_map_local.at(w_point);
			if (goal_node_t != w_point)
			{
				double cost_wt = CalculateEdgeCost(w_point, goal_node_t, obstacles);
				if (cost_wt < std::numeric_limits<double>::max())
				{
					boost::add_edge(w_vertex, t_vertex_local, cost_wt, G1_local);
				}
			}
		}
	}

	return { G1_local, point_to_vertex_map_local };
}

GraphData ConstructG2_Graph(
	const Point& start_node_s,
	const Point& goal_node_t,
	const std::vector<Polygon>& obstacles,
	const VoronoiData& voronoi_data)
{
	using namespace boost_compat;

	std::vector<Point> graph_points_collector;
	graph_points_collector.push_back(start_node_s);
	graph_points_collector.push_back(goal_node_t);

	// Вершины G2 - это s, t и все вершины $\tilde{\mathcal{V}}$
	// (уникальные конечные точки ребер refined_edges)
	for (const auto& [p1, p2] : voronoi_data.refined_edges)
	{
		graph_points_collector.push_back(p1);
		graph_points_collector.push_back(p2);
	}
	// Вершины ячеек также являются частью V(tilde_V) и уже должны быть учтены выше.
	// Добавление их из voronoi_data.all_tilde_V_cells ниже - для полноты, если
	// refined_edges могли не покрыть какие-то изолированные вершины (маловероятно).
	for (const auto& cell_T_poly : voronoi_data.all_tilde_V_cells)
	{
		for (const auto& v_of_T : cell_T_poly)
		{
			graph_points_collector.push_back(v_of_T);
		}
	}

	std::sort(graph_points_collector.begin(), graph_points_collector.end());
	graph_points_collector.erase(
		std::ranges::unique(graph_points_collector).begin(),
		graph_points_collector.end());

	std::map<Point, Vertex> point_to_vertex_map_local;
	Graph G2_local(graph_points_collector.size());

	for (size_t i = 0; i < graph_points_collector.size(); ++i)
	{
		Vertex v_desc = vertex(i, G2_local);
		point_to_vertex_map_local[graph_points_collector[i]] = v_desc;
		G2_local[v_desc] = graph_points_collector[i];
	}

	// Добавляем ребра для G2
	// Для каждой ячейки T в voronoi_data.all_tilde_V_cells:
	for (const auto& cell_T_polygon : voronoi_data.all_tilde_V_cells)
	{
		const Polygon& V_T = cell_T_polygon; // V_T - это вершины ячейки T (Polygon = std::vector<Point>)

		// Добавляем ребра между всеми парами РАЗЛИЧНЫХ вершин u, v из V_T
		for (size_t i = 0; i < V_T.size(); ++i)
		{
			for (size_t j = i + 1; j < V_T.size(); ++j)
			{
				// j = i + 1 чтобы избежать дублей и петель u-u
				const Point& u_point = V_T[i];
				const Point& v_point = V_T[j];

				if (point_to_vertex_map_local.contains(u_point) && point_to_vertex_map_local.contains(v_point))
				{
					Vertex u_vertex = point_to_vertex_map_local.at(u_point);
					Vertex v_vertex = point_to_vertex_map_local.at(v_point);

					double cost = CalculateEdgeCost(u_point, v_point, obstacles);
					if (cost < std::numeric_limits<double>::max())
					{
						// Это ребро может дублировать ребро из refined_edges, если u,v были смежными.
						// add_edge в Boost (для undirectedS) не создаст параллельных ребер,
						// но если вес отличается, это не определено.
						// Для простоты добавляем; Дейкстра выберет лучший путь.
						// Либо можно проверять, существует ли уже ребро с меньшим весом.
						boost::add_edge(u_vertex, v_vertex, cost, G2_local);
					}
				}
			}
		}
	}

	// Соединения для s, если s находится в известной ячейке
	if (voronoi_data.s_cell_idx != -1 && static_cast<size_t>(voronoi_data.s_cell_idx) < voronoi_data.all_tilde_V_cells.size())
	{
		const Polygon& V_Ts = voronoi_data.all_tilde_V_cells[voronoi_data.s_cell_idx]; // Вершины ячейки T_s
		Vertex s_vertex = point_to_vertex_map_local.at(start_node_s); // Получаем Vertex для s
		for (const auto& u_point_in_Ts : V_Ts)
		{
			if (point_to_vertex_map_local.contains(u_point_in_Ts) && start_node_s != u_point_in_Ts)
			{
				Vertex u_vertex = point_to_vertex_map_local.at(u_point_in_Ts);
				double cost_su = CalculateEdgeCost(start_node_s, u_point_in_Ts, obstacles);
				if (cost_su < std::numeric_limits<double>::max())
				{
					boost::add_edge(s_vertex, u_vertex, cost_su, G2_local);
				}
			}
		}
	}

	// Соединения для t, если t находится в известной ячейке
	if (voronoi_data.t_cell_idx != -1 && static_cast<size_t>(voronoi_data.t_cell_idx) < voronoi_data.all_tilde_V_cells.size())
	{
		const Polygon& V_Tt = voronoi_data.all_tilde_V_cells[voronoi_data.t_cell_idx]; // Вершины ячейки T_t
		Vertex t_vertex = point_to_vertex_map_local.at(goal_node_t); // Получаем Vertex для t
		for (const auto& u_point_in_Tt : V_Tt)
		{
			if (point_to_vertex_map_local.contains(u_point_in_Tt) && goal_node_t != u_point_in_Tt)
			{
				Vertex u_vertex = point_to_vertex_map_local.at(u_point_in_Tt);
				double cost_ut = CalculateEdgeCost(u_point_in_Tt, goal_node_t, obstacles);
				if (cost_ut < std::numeric_limits<double>::max())
				{
					boost::add_edge(u_vertex, t_vertex, cost_ut, G2_local);
				}
			}
		}
	}

	return { G2_local, point_to_vertex_map_local };
}

std::vector<Point> RunDijkstraAlgorithm(
	const boost_compat::Graph& graph,
	const std::map<Point, boost_compat::Vertex>& point_to_vertex_map,
	const Point& start_node,
	const Point& goal_node)
{
	using namespace boost_compat;

	if (!point_to_vertex_map.contains(start_node) || !point_to_vertex_map.contains(goal_node))
	{
		return {};
	}

	Vertex s_vertex = point_to_vertex_map.at(start_node);
	Vertex t_vertex = point_to_vertex_map.at(goal_node);

	DijkstraShortestPath dj(graph);
	return dj.FindPath(s_vertex, t_vertex);
}

std::vector<Point> FindPath(
	GraphType type,
	const VoronoiData& voronoi_data,
	const Point& start_node_s,
	const Point& goal_node_t,
	const std::vector<Polygon>& obstacles)
{
	using namespace boost_compat;

	std::vector<Point> N_s_for_G1;
	if (voronoi_data.s_cell_idx < voronoi_data.all_tilde_V_cells.size())
	{
		N_s_for_G1 = voronoi_data.all_tilde_V_cells[voronoi_data.s_cell_idx];
	}

	std::vector<Point> N_t_for_G1;
	if (voronoi_data.t_cell_idx < voronoi_data.all_tilde_V_cells.size())
	{
		N_t_for_G1 = voronoi_data.all_tilde_V_cells[voronoi_data.t_cell_idx];
	}
	if (type == GraphType::G1)
	{
		GraphData g1_construction_result = ConstructG1_Graph(
			voronoi_data.refined_edges, start_node_s, goal_node_t, obstacles, N_s_for_G1, N_t_for_G1);
		const Graph& G1 = g1_construction_result.graph;
		auto& g1_point_to_vertex_map = g1_construction_result.point_to_vertex_map;

		auto path = RunDijkstraAlgorithm(G1, g1_point_to_vertex_map, start_node_s, goal_node_t);
		return path;
	}

	if (type == GraphType::G2)
	{
		GraphData g2_construction_result = ConstructG2_Graph(
			start_node_s, goal_node_t, obstacles, voronoi_data);
		const Graph& G2 = g2_construction_result.graph;
		auto& g2_point_to_vertex_map = g2_construction_result.point_to_vertex_map;

		auto path = RunDijkstraAlgorithm(G2, g2_point_to_vertex_map, start_node_s, goal_node_t);
		return path;
	}

	return {};
}
} // namespace

tp::PathFinder::PathFinder(
	GraphType type,
	const VoronoiData& voronoi_data,
	const std::vector<shapes::Polygon>& obstacles,
	const shapes::Point& start, const shapes::Point& end)
{
	m_path = FindPath(type, voronoi_data, start, end, obstacles);
}

std::vector<Segment> PathFinder::GetPath() const
{
	std::vector<Segment> segPath;
	if (m_path.size() < 2)
	{
		return segPath;
	}

	for (size_t i = 0; i < m_path.size() - 1; ++i)
	{
		auto p1 = m_path[i];
		auto p2 = m_path[i + 1];
		segPath.emplace_back(Segment{ p1, p2 });
	}

	return segPath;
}
