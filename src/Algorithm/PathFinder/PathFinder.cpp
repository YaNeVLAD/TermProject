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
    using namespace tp::shapes; // Для Point, Vec2D, Polygon
    using namespace tp::math;   // Для Distance, DistanceToSegmentSquared

    float segment_length = Distance(p1, p2);
    if (segment_length < FLT_EPSILON)
    {
       return 0.0; // Нулевая стоимость для ребер нулевой длины
    }

    // Функция для вычисления зазора в точке p (остается без изменений)
    auto get_clearance = [&](const Point& p) -> float {
       float min_dist_sq = std::numeric_limits<float>::max();
       if (obstacles.empty()) {
          // Возвращаем очень большое значение, если нет препятствий,
          // чтобы 1/clr было очень маленьким, но не нулем.
          // Либо можно считать зазор "бесконечным", и тогда стоимость пути в свободном пространстве стремится к 0,
          // что не совсем то, что нужно для формулы.
          // Безопаснее вернуть большое число, соответствующее "очень безопасно".
          // Или, если интерпретировать "нет препятствий" как идеальные условия,
          // то 1/clr должно быть минимально возможным положительным числом.
          // Для практических целей, если препятствий нет, может быть, эта функция стоимости не так важна
          // или должна быть заменена на простое расстояние.
          // Пока оставим как есть, но это пограничный случай.
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
             // Предполагается, что math::DistanceToSegmentSquared существует и корректна
             float d_sq = DistanceToSegmentSquared(p, v1, v2);
             if (d_sq < min_dist_sq)
             {
                min_dist_sq = d_sq;
             }
          }
       }

       if (min_dist_sq < FLT_EPSILON * FLT_EPSILON) // Сравниваем с квадратом эпсилон для большей точности
          return FLT_EPSILON; // Избегаем деления на ноль и очень малых зазоров
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

       if (clearance_at_step_start < FLT_EPSILON) {
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

std::optional<Point> FindNearestPoint(const Point& point, const std::vector<Point>& points, const std::vector<Polygon>& obstacles)
{
	std::optional<Point> nearestPoint = std::nullopt;
	auto minCost = std::numeric_limits<double>::max();

	for (const auto& p : points)
	{
		if (point == p)
		{
			continue;
		}

		if (double currentCost = CalculateEdgeCost(p, point, obstacles); currentCost < minCost)
		{
			minCost = currentCost;
			nearestPoint = p;
		}
	}

	return nearestPoint;
}

std::vector<Point> FindPath(
    const std::vector<Segment>& segments_tilde_V, // Ребра $\tilde{\mathcal{V}}$
    const Point& start_node_s,
    const Point& goal_node_t,
    const std::vector<Polygon>& obstacles,
    const std::vector<Point>& N_s, // Вершины ячейки T_s, содержащей start_node_s
    const std::vector<Point>& N_t  // Вершины ячейки T_t, содержащей goal_node_t
)
{
    using namespace boost_compat; // Для Graph, Vertex
    using shapes::Point;
    using shapes::Segment;

    std::vector<Point> graph_points_collector;
    graph_points_collector.push_back(start_node_s);
    graph_points_collector.push_back(goal_node_t);

    for (const auto& [p1, p2] : segments_tilde_V) {
        graph_points_collector.push_back(p1);
        graph_points_collector.push_back(p2);
    }
    // Вершины из N_s и N_t уже должны быть конечными точками каких-либо ребер в segments_tilde_V,
    // но для надежности можно добавить их явно, если они могут быть не включены.
    // В определении G1 они являются частью V(tilde_V).
    for (const auto& p_ns : N_s) { graph_points_collector.push_back(p_ns); }
    for (const auto& p_nt : N_t) { graph_points_collector.push_back(p_nt); }

    std::sort(graph_points_collector.begin(), graph_points_collector.end());
    graph_points_collector.erase(
        std::ranges::unique(graph_points_collector).begin(),
        graph_points_collector.end()
    );

    std::map<Point, Vertex> point_to_vertex_map;
    Graph G1(graph_points_collector.size());

    for (size_t i = 0; i < graph_points_collector.size(); ++i) {
        Vertex v_desc = boost::vertex(i, G1); // Получаем дескриптор вершины
        point_to_vertex_map[graph_points_collector[i]] = v_desc;
        G1[v_desc] = graph_points_collector[i]; // Присваиваем свойство (точку) вершине
    }

    Vertex s_vertex = point_to_vertex_map.at(start_node_s);
    Vertex t_vertex = point_to_vertex_map.at(goal_node_t);

    // 1. Добавляем ребра (w, w') для отрезков из segments_tilde_V (ребра $\tilde{\mathcal{V}}$)
    for (const auto& [p1, p2] : segments_tilde_V) {
        // Проверяем, что обе точки сегмента есть в нашей карте вершин
        // (должны быть, так как мы их собирали в graph_points_collector)
        if (point_to_vertex_map.contains(p1) && point_to_vertex_map.contains(p2) && p1 != p2) {
            Vertex u = point_to_vertex_map.at(p1);
            Vertex v = point_to_vertex_map.at(p2);
            double cost = CalculateEdgeCost(p1, p2, obstacles);
            // Добавляем ребро, только если стоимость не бесконечна (т.е. путь не через препятствие)
            if (cost < std::numeric_limits<double>::max()) {
                boost::add_edge(u, v, cost, G1);
            }
        }
    }

    // 2. Добавляем ребра (s, w) для всех w из N_s
    for (const auto& w_point : N_s) {
        if (point_to_vertex_map.contains(w_point)) { // w_point должна быть вершиной из $\tilde{\mathcal{V}}$
            Vertex w_vertex = point_to_vertex_map.at(w_point);
            // Избегаем петель, если s сама является вершиной своей ячейки (маловероятно, но возможно)
            if (start_node_s != w_point) {
                double cost_sw = CalculateEdgeCost(start_node_s, w_point, obstacles);
                if (cost_sw < std::numeric_limits<double>::max()) {
                    boost::add_edge(s_vertex, w_vertex, cost_sw, G1);
                }
            }
        }
    }

    // 3. Добавляем ребра (w, t) для всех w из N_t
    for (const auto& w_point : N_t) {
        if (point_to_vertex_map.contains(w_point)) { // w_point должна быть вершиной из $\tilde{\mathcal{V}}$
            Vertex w_vertex = point_to_vertex_map.at(w_point);
            if (goal_node_t != w_point) {
                double cost_wt = CalculateEdgeCost(w_point, goal_node_t, obstacles);
                if (cost_wt < std::numeric_limits<double>::max()) {
                    boost::add_edge(w_vertex, t_vertex, cost_wt, G1);
                }
            }
        }
    }

    // Опциональное прямое ребро (s, t)
    // Стоит добавить, если N_s или N_t пусты, или если s и t в одной ячейке (тогда t \in N_s).
    // Dijkstra сам выберет этот путь, если он оптимален.
	if (N_s.empty() && N_t.empty())
	{
		double direct_cost_st = CalculateEdgeCost(start_node_s, goal_node_t, obstacles);
		if (direct_cost_st < std::numeric_limits<double>::max()){
			// Проверяем, что s_vertex и t_vertex различны, чтобы избежать петли, если start == goal
			if (start_node_s != goal_node_t) {
				boost::add_edge(s_vertex, t_vertex, direct_cost_st, G1);
			}
		}
	}

	DijkstraShortestPath dj(G1);
	auto path = dj.FindPath(point_to_vertex_map[start_node_s], point_to_vertex_map[goal_node_t]);
	return path;
}
} // namespace

PathFinder::PathFinder(
	const std::vector<Segment>& segments,
	const std::vector<Polygon>& obstacles,
	const Point& start, const Point& end,
	const std::vector<Point>& N_s,
	const std::vector<Point>& N_t)
	: m_path(FindPath(segments, start, end, obstacles, N_s, N_t))
{
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