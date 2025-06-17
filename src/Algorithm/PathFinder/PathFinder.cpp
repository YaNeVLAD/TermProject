//
// Created by User on 15.04.2025.
//

#include "PathFinder.h"

#include "../../Utility/Logger/Logger.h"
#include "../Math/Math.hpp"
#include "../Structures/BoostCompat.h"

using namespace tp;
using namespace tp::math;
using namespace tp::shapes;

namespace
{
double CalculateEdgeCost(const Point& p1, const Point& p2,
	const std::vector<Polygon>& obstacles)
{
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
			for (size_t i = 0; i < polygon.size() - 1; ++i)
			{
				size_t curr = i;
				size_t next = i + 1;
				const auto& v1 = polygon[curr];
				const auto& v2 = polygon[next];
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

bool IsAnyContains(const Point& point, const std::vector<Polygon>& obstacles)
{
	bool contains = false;
	for (const auto& obstacle : obstacles)
	{
		if (Contains(point, obstacle))
		{
			contains = true;
			break;
		}
	}
	return contains;
}

double CalculatePathCost(const std::vector<Point>& path, const std::vector<Polygon>& obstacles)
{
	double total_cost = 0.0;
	if (path.size() < 2)
		return 0.0;

	for (size_t i = 0; i < path.size() - 1; ++i)
	{
		total_cost += CalculateEdgeCost(path[i], path[i + 1], obstacles); // Reuse your existing cost function
	}
	return total_cost;
}

// Function to calculate clearance along a Bezier curve (sampled)
float GetClearanceAlongBezier(const Point& p0, const Point& p1, const Point& p2, const std::vector<Polygon>& obstacles, int num_samples = 20)
{
	float min_clearance = std::numeric_limits<float>::max();

	for (int i = 0; i <= num_samples; ++i)
	{
		float t = static_cast<float>(i) / static_cast<float>(num_samples);
		// Quadratic Bezier: B(t) = (1-t)^2 * P0 + 2(1-t)t * P1 + t^2 * P2
		auto sample_vec = (1.0f - t) * (1.0f - t) * p0 + 2.0f * (1.0f - t) * t * p1 + t * t * p2;
		Point sample_point = { sample_vec.x, sample_vec.y };

		float current_clearance = std::numeric_limits<float>::max();
		if (obstacles.empty())
		{
			return std::sqrt(std::numeric_limits<float>::max()); // Or some large value
		}

		for (const auto& polygon : obstacles)
		{
			if (polygon.empty())
				continue;
			for (size_t j = 0; j < polygon.size() - 1; ++j)
			{
				size_t curr = j;
				size_t next = j + 1;
				const auto& v1 = polygon[curr];
				const auto& v2 = polygon[next];
				float d_sq = DistanceToSegmentSquared(sample_point, v1, v2);
				current_clearance = std::min(current_clearance, std::sqrt(d_sq));
			}
			// Also check distance to vertices of the polygon
			for (const auto& v : polygon)
			{
				current_clearance = std::min(current_clearance, Distance(sample_point, v));
			}
		}
		min_clearance = std::min(min_clearance, current_clearance);

		// Early exit if collision detected or clearance is too low
		if (min_clearance < FLT_EPSILON)
			return FLT_EPSILON;
	}
	return min_clearance;
}

// Function to check if a Bezier curve intersects any obstacle
bool IsBezierColliding(const Point& p0, const Point& p1, const Point& p2, const std::vector<Polygon>& obstacles, int num_samples = 20)
{
	// Simplified collision check: sample points and check if any are inside an obstacle or too close
	// For robust collision, you'd need more sophisticated methods (e.g., using AABB or OBB of the curve segment, or
	// a sweep test, or direct curve-polygon intersection tests).
	for (int i = 0; i <= num_samples; ++i)
	{
		float t = static_cast<float>(i) / static_cast<float>(num_samples);
		auto sample_vec = (1.0f - t) * (1.0f - t) * p0 + 2.0f * (1.0f - t) * t * p1 + t * t * p2;
		Point sample_point = { sample_vec.x, sample_vec.y };

		if (IsAnyContains(sample_point, obstacles)) // Reuse your IsAnyContains
		{
			return true;
		}

		// Also check if clearance is too low (practically a collision)
		if (GetClearanceAlongBezier(p0, p1, p2, obstacles, num_samples) < FLT_EPSILON)
		{
			return true;
		}
	}
	return false;
}

// Smoothes the path by replacing sharp corners with Bezier curves
std::vector<Point> SmoothPath(
	const std::vector<Point>& original_path,
	const std::vector<Polygon>& obstacles,
	const VoronoiData& voronoi_data)
{
	if (original_path.size() < 3)
	{
		return original_path; // Cannot smooth a path with less than 3 points
	}

	std::vector<Point> smoothed_path;
	smoothed_path.push_back(original_path.front()); // Start point is always fixed

	for (size_t i = 1; i < original_path.size() - 1; ++i)
	{
		const Point& p_prev = original_path[i - 1];
		const Point& p_curr = original_path[i];
		const Point& p_next = original_path[i + 1];

		// Try to smooth the corner at p_curr using a quadratic Bezier curve
		// The control points are p_prev, p_curr, p_next
		// This simple Bezier will pass through p_prev and p_next, and its tangent at p_prev will be along p_prev-p_curr,
		// and its tangent at p_next will be along p_curr-p_next.
		// The curve will 'bend' towards p_curr.

		// A more advanced smoothing would involve calculating new control points that
		// pull the curve away from obstacles, potentially using information about
		// the Voronoi cell p_curr is in.

		// For a basic quadratic Bezier, the intermediate points are on the curve.
		// We'll replace the two segments (p_prev, p_curr) and (p_curr, p_next)
		// with the Bezier curve if it's collision-free and has better/acceptable cost.

		// Define a simple Bezier that uses a "midpoint" as a control point
		// This is not a standard Bezier, but a common trick for corner rounding.
		// A better approach for "smoothing" in the sense of the paper
		// would involve finding a control point that pulls the curve into higher clearance regions.

		// Let's use p_curr as the 'main' control point for a quadratic Bezier
		// from a point near p_prev on (p_prev, p_curr) to a point near p_next on (p_curr, p_next).
		// This is just one way to implement corner smoothing.

		// Option 1: Simple quadratic Bezier through p_prev, p_curr, p_next
		// Points on this curve are: B(t) = (1-t)^2 * p_prev + 2(1-t)t * p_curr + t^2 * p_next
		// This replaces the *entire* path segment from p_prev to p_next via p_curr.
		// This might be too aggressive if p_prev and p_next are far apart.

		// Let's try a simpler corner-rounding:
		// Create a small arc or Bezier segment that transitions from the incoming to outgoing segment.
		// We need to define two points `s1` and `s2` on the segments `p_prev-p_curr` and `p_curr-p_next` respectively.
		// Then, we'll try to insert a curve between `s1` and `s2`, with `p_curr` potentially as a control point.

		float smoothing_radius = 25.f; // Adjust this based on desired smoothness and clearance
		// Calculate points s1 and s2 along the segments, away from p_curr
		auto s1v = (Vec2D)p_curr - (p_curr - p_prev).Normalize() * smoothing_radius;
		auto s2v = (Vec2D)p_curr - (p_curr - p_next).Normalize() * smoothing_radius;

		Point s1 = { s1v.x, s1v.y };
		Point s2 = { s2v.x, s2v.y };

		// Ensure s1 and s2 don't go past p_prev or p_next
		if (Distance(p_prev, s1) < FLT_EPSILON)
			s1 = p_prev;
		if (Distance(p_next, s2) < FLT_EPSILON)
			s2 = p_next;

		// Option 2: Quadratic Bezier between s1 and s2, using p_curr as the control point
		// B(t) = (1-t)^2 * s1 + 2(1-t)t * p_curr + t^2 * s2
		// This curve replaces the corner part of the path.
		// The path becomes: ... p_prev -> s1 -> (Bezier curve) -> s2 -> p_next ...

		bool can_smooth = true;
		// Check if the Bezier curve is collision-free and maintains sufficient clearance
		if (IsBezierColliding(s1, p_curr, s2, obstacles))
		{
			can_smooth = false;
		}

		if (can_smooth)
		{
			// Add points along the Bezier curve to the smoothed path
			smoothed_path.push_back(s1);
			int num_bezier_segments = 10; // Number of straight line segments to approximate Bezier
			for (int j = 1; j <= num_bezier_segments; ++j)
			{
				float t = static_cast<float>(j) / static_cast<float>(num_bezier_segments);
				auto bezier_vec = (1.0f - t) * (1.0f - t) * s1 + 2.0f * (1.0f - t) * t * p_curr + t * t * s2;
				Point bezier_point = { bezier_vec.x, bezier_vec.y };
				smoothed_path.push_back(bezier_point);
			}
		}
		else
		{
			// If smoothing not possible, just add the current point
			smoothed_path.push_back(p_curr);
		}
	}

	smoothed_path.push_back(original_path.back()); // End point is always fixed

	return smoothed_path;
}

GraphData ConstructG1_Graph(
	const std::vector<Segment>& segments_tilde_V,
	const Point& start_node_s,
	const Point& goal_node_t,
	const std::vector<Polygon>& obstacles,
	const Polygon& N_s,
	const Polygon& N_t)
{
	using namespace boost_compat;

	std::vector<Point> graph_points_collector;
	graph_points_collector.push_back(start_node_s);
	graph_points_collector.push_back(goal_node_t);

	for (const auto& [p1, p2, color] : segments_tilde_V)
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

	std::ranges::sort(graph_points_collector);
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
	for (const auto& [p1, p2, color] : segments_tilde_V)
	{
		if (point_to_vertex_map_local.contains(p1) && point_to_vertex_map_local.contains(p2) && p1 != p2)
		{
			Vertex u = point_to_vertex_map_local.at(p1);
			Vertex v = point_to_vertex_map_local.at(p2);
			double cost = CalculateEdgeCost(p1, p2, obstacles);
			if (cost == std::numeric_limits<double>::max())
			{
				LOG_DEBUG(std::format("Max cost edge detected: [{}, {}],[{}, {}]",
					p1.x, p1.y, p2.x, p2.y));
			}

			if (cost < std::numeric_limits<double>::max())
			{
				boost::add_edge(u, v, cost, G1_local);
			}
		}
	}

	// 2. Добавляем ребра (s, w) для w из N_s
	for (const auto& w_point : N_s)
	{
		if (point_to_vertex_map_local.contains(w_point))
		{
			Vertex w_vertex = point_to_vertex_map_local.at(w_point);
			if (start_node_s != w_point)
			{
				double cost_sw = CalculateEdgeCost(start_node_s, w_point, obstacles);

				if (cost_sw == std::numeric_limits<double>::max())
				{
					LOG_DEBUG(std::format("Max cost edge detected: [{}, {}],[{}, {}]",
						start_node_s.x, start_node_s.y, w_point.x, w_point.y));
				}

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
				if (cost_wt == std::numeric_limits<double>::max())
				{
					LOG_DEBUG(std::format("Max cost edge detected: [{}, {}],[{}, {}]",
						start_node_s.x, start_node_s.y, w_point.x, w_point.y));
				}

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
	for (const auto& [p1, p2, color] : voronoi_data.refined_edges)
	{
		graph_points_collector.push_back(p1);
		graph_points_collector.push_back(p2);
	}

	std::ranges::sort(graph_points_collector);
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

					if (cost == std::numeric_limits<double>::max())
					{
						LOG_DEBUG(std::format("Max cost edge detected: [{}, {}],[{}, {}]",
							u_point.x, u_point.y, v_point.x, v_point.y));
					}

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
	if (voronoi_data.s_cell_idx != -1 && voronoi_data.s_cell_idx < voronoi_data.all_tilde_V_cells.size())
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
	if (voronoi_data.t_cell_idx != -1 && voronoi_data.t_cell_idx < voronoi_data.all_tilde_V_cells.size())
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
	const std::vector<Polygon>& obstacles,
	std::vector<Segment>& edges)
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

	std::vector<Point> path;

	if (type == GraphType::G1)
	{
		GraphData g1_construction_result = ConstructG1_Graph(
			voronoi_data.refined_edges, start_node_s, goal_node_t, obstacles, N_s_for_G1, N_t_for_G1);
		const Graph& G1 = g1_construction_result.graph;
		auto& g1_point_to_vertex_map = g1_construction_result.point_to_vertex_map;

		edges = GetAllEdges(G1);

		path = RunDijkstraAlgorithm(G1, g1_point_to_vertex_map, start_node_s, goal_node_t);
	}

	if (type == GraphType::G2)
	{
		GraphData g2_construction_result = ConstructG2_Graph(
			start_node_s, goal_node_t, obstacles, voronoi_data);
		const Graph& G2 = g2_construction_result.graph;
		auto& g2_point_to_vertex_map = g2_construction_result.point_to_vertex_map;

		edges = GetAllEdges(G2);

		path = RunDijkstraAlgorithm(G2, g2_point_to_vertex_map, start_node_s, goal_node_t);
	}

	if (!path.empty())
	{
		path = SmoothPath(path, obstacles, voronoi_data);
	}

	return path;
}
} // namespace

PathFinder::PathFinder(
	GraphType type,
	const VoronoiData& voronoi_data,
	const std::vector<Polygon>& obstacles,
	const Point& start, const Point& end)
{
	m_path = FindPath(type, voronoi_data, start, end, obstacles, m_edges);
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

std::vector<shapes::Segment> tp::PathFinder::GetEdges() const
{
	return m_edges;
}
