//
// Created by User on 15.04.2025.
//

#include "PathFinder.h"

#include <cmath>

#include "../../Utility/Logger/Logger.h"
#include "../Math/Math.hpp"
#include "../Structures/BoostCompat.h"

using namespace tp;
using namespace tp::math;
using namespace tp::shapes;

namespace
{
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

float GetClearanceAlongBezier(const Point& p0, const Point& p1, const Point& p2, const std::vector<Polygon>& obstacles, int num_samples = 20)
{
	float min_clearance = std::numeric_limits<float>::max();

	for (int i = 0; i <= num_samples; ++i)
	{
		float t = static_cast<float>(i) / static_cast<float>(num_samples);
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
			for (const auto& v : polygon)
			{
				current_clearance = std::min(current_clearance, Distance(sample_point, v));
			}
		}
		min_clearance = std::min(min_clearance, current_clearance);

		if (min_clearance < FLT_EPSILON)
			return FLT_EPSILON;
	}
	return min_clearance;
}

bool IsBezierColliding(const Point& p0, const Point& p1, const Point& p2, const std::vector<Polygon>& obstacles, int num_samples = 20)
{
	for (int i = 0; i <= num_samples; ++i)
	{
		float t = static_cast<float>(i) / static_cast<float>(num_samples);
		auto sample_vec = (1.0f - t) * (1.0f - t) * p0 + 2.0f * (1.0f - t) * t * p1 + t * t * p2;
		Point sample_point = { sample_vec.x, sample_vec.y };

		if (IsAnyContains(sample_point, obstacles))
		{
			return true;
		}

		if (GetClearanceAlongBezier(p0, p1, p2, obstacles, num_samples) < FLT_EPSILON)
		{
			return true;
		}
	}
	return false;
}

std::vector<Point> SmoothPath(
	const std::vector<Point>& original_path,
	const std::vector<Polygon>& obstacles,
	const VoronoiData& voronoi_data)
{
	if (original_path.size() < 3)
	{
		return original_path;
	}

	std::vector<Point> smoothed_path;
	smoothed_path.push_back(original_path.front());

	for (size_t i = 1; i < original_path.size() - 1; ++i)
	{
		const Point& p_prev = original_path[i - 1];
		const Point& p_curr = original_path[i];
		const Point& p_next = original_path[i + 1];

		float smoothing_radius = std::clamp(
			std::min(CalculateEdgeCost(p_prev, p_curr, obstacles), CalculateEdgeCost(p_next, p_curr, obstacles)) * 100.f, 0.0, 25.0);

		float actual_smoothing_radius = smoothing_radius;

		Vec2D vec_prev_to_curr = p_curr - p_prev;
		Vec2D vec_curr_to_next = p_next - p_curr;

		Vec2D norm_prev_to_curr = vec_prev_to_curr.Normalize();
		Vec2D norm_curr_to_next = vec_curr_to_next.Normalize();

		Point s1 = p_curr - norm_prev_to_curr.Scale(actual_smoothing_radius);
		Point s2 = p_curr + norm_curr_to_next.Scale(actual_smoothing_radius);

		if (Distance(s1, p_curr) > Distance(p_prev, p_curr))
			s1 = p_prev;
		if (Distance(s2, p_curr) > Distance(p_next, p_curr))
			s2 = p_next;

		auto s1v = (Vec2D)p_curr - (p_curr - p_prev).Normalize() * smoothing_radius;
		auto s2v = (Vec2D)p_curr - (p_curr - p_next).Normalize() * smoothing_radius;

		bool canSmooth = !IsBezierColliding(s1, p_curr, s2, obstacles);

		if (canSmooth)
		{
			smoothed_path.push_back(s1);
			int num_bezier_segments = 10;
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
			smoothed_path.push_back(p_curr);
		}
	}

	smoothed_path.push_back(original_path.back());

	return smoothed_path;
}

std::vector<Point> InterpolatePoints(const Point& p1, const Point& p2, double segment_length_threshold)
{
	std::vector<Point> interpolated_points;
	interpolated_points.push_back(p1);

	double dx = p2.x - p1.x;
	double dy = p2.y - p1.y;
	double segment_length = std::sqrt(dx * dx + dy * dy);

	if (segment_length > segment_length_threshold && segment_length_threshold > 0)
	{
		int num_segments = static_cast<int>(std::ceil(segment_length / segment_length_threshold));
		for (int i = 1; i < num_segments; ++i)
		{
			double t = static_cast<double>(i) / num_segments;
			interpolated_points.emplace_back(p1.x + t * dx, p1.y + t * dy);
		}
	}
	interpolated_points.push_back(p2);
	return interpolated_points;
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

	// (w, w') для segments_tilde_V
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

	// (s, w) для w из N_s
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

	// (w, t) для w из N_t
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
	double refined_edge_segment_threshold = 25;

	using namespace boost_compat;

	std::vector<Point> graph_points_collector;
	graph_points_collector.push_back(start_node_s);
	graph_points_collector.push_back(goal_node_t);

	for (const auto& [p1, p2, color] : voronoi_data.refined_edges)
	{
		std::vector<Point> interpolated = InterpolatePoints(p1, p2, refined_edge_segment_threshold);
		for (const auto& p : interpolated)
		{
			graph_points_collector.push_back(p);
		}
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

	for (const auto& [p1_orig, p2_orig, color] : voronoi_data.refined_edges)
	{
		std::vector<Point> interpolated = InterpolatePoints(p1_orig, p2_orig, refined_edge_segment_threshold);
		for (size_t i = 0; i < interpolated.size() - 1; ++i)
		{
			const Point& u_point = interpolated[i];
			const Point& v_point = interpolated[i + 1];

			if (point_to_vertex_map_local.contains(u_point) && point_to_vertex_map_local.contains(v_point))
			{
				Vertex u_vertex = point_to_vertex_map_local.at(u_point);
				Vertex v_vertex = point_to_vertex_map_local.at(v_point);

				double cost = CalculateEdgeCost(u_point, v_point, obstacles);

				if (cost == std::numeric_limits<double>::max())
				{
					LOG_DEBUG(std::format("Max cost edge detected (interpolated): [{}, {}],[{}, {}]",
						u_point.x, u_point.y, v_point.x, v_point.y));
				}

				if (cost < std::numeric_limits<double>::max())
				{
					boost::add_edge(u_vertex, v_vertex, cost, G2_local);
				}
			}
		}
	}

	for (const auto& cell_T_polygon : voronoi_data.all_tilde_V_cells)
	{
		const Polygon& V_T = cell_T_polygon;

		for (size_t i = 0; i < V_T.size(); ++i)
		{
			for (size_t j = i + 1; j < V_T.size(); ++j)
			{
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
						boost::add_edge(u_vertex, v_vertex, cost, G2_local);
					}
				}
			}
		}
	}

	if (voronoi_data.s_cell_idx != -1 && voronoi_data.s_cell_idx < voronoi_data.all_tilde_V_cells.size())
	{
		const Polygon& V_Ts = voronoi_data.all_tilde_V_cells[voronoi_data.s_cell_idx];
		Vertex s_vertex = point_to_vertex_map_local.at(start_node_s);
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

std::vector<Point> ProcessPath(
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
		path = SmoothPath(path, obstacles, voronoi_data);
	}

	if (type == GraphType::G2)
	{
		GraphData g2_construction_result = ConstructG2_Graph(
			start_node_s, goal_node_t, obstacles, voronoi_data);
		const Graph& G2 = g2_construction_result.graph;
		auto& g2_point_to_vertex_map = g2_construction_result.point_to_vertex_map;

		edges = GetAllEdges(G2);

		path = RunDijkstraAlgorithm(G2, g2_point_to_vertex_map, start_node_s, goal_node_t);
		path = SmoothPath(path, obstacles, voronoi_data);
	}

	return path;
}
} // namespace

std::vector<shapes::Segment> tp::PathFinder::FindPath(
	GraphType type,
	const VoronoiData& voronoi_data,
	const std::vector<shapes::Polygon>& obstacles,
	const shapes::Point& start, const shapes::Point& end)
{
	m_path = ProcessPath(type, voronoi_data, start, end, obstacles, m_edges);
	return GetPath();
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
		auto& p1 = m_path[i];
		auto& p2 = m_path[i + 1];
		segPath.emplace_back(Segment{ p1, p2 });
	}

	return segPath;
}

std::vector<shapes::Segment> tp::PathFinder::GetEdges() const
{
	return m_edges;
}
