//
// Created by User on 19.04.2025.
//

#include "Voronoi.h"

#include <boost/geometry.hpp>
#include <boost/polygon/voronoi.hpp>

#include "../../Utility/Logger/Logger.h"
#include "../Math/Math.hpp"
#include "../PathFinder/PathFinder.h"
#include "../Structures/BoostCompat.h"
#include "Helpers.h"

#include <boost/geometry.hpp>

#include <format>
#include <optional>

using namespace tp;
using namespace tp::shapes;

struct ObstacleFeature
{
	enum class Type
	{
		VERTEX,
		SEGMENT
	};

	Type type;
	Point vertex_site; // Значимо, если type == VERTEX
	Segment segment_site; // Значимо, если type == SEGMENT

	explicit ObstacleFeature(Point vertex)
		: type(Type::VERTEX)
		, vertex_site(vertex)
	{
	}

	explicit ObstacleFeature(Segment segment)
		: type(Type::SEGMENT)
		, segment_site(segment)
	{
	}

	// $\psi_o(x)$ - возвращает ближайшую точку на данной особенности 'o' к точке 'x'
	Point getPsi(const Point& p) const
	{
		if (type == Type::VERTEX)
		{
			return vertex_site;
		}
		// Type::SEGMENT
		const Vec2D ab = segment_site.p2 - segment_site.p1;
		const auto [x, y] = p - segment_site.p1; // Вектор от начала сегмента до точки x

		const float ab_len_sq = ab.LengthSquared();

		// Если сегмент вырожден в точку
		if (ab_len_sq < FLT_EPSILON * FLT_EPSILON)
		{
			return segment_site.p1;
		}

		// t = dot(ax, ab) / dot(ab, ab)
		float t = (x * ab.x + y * ab.y) / ab_len_sq;

		if (t < 0.0f)
		{
			return segment_site.p1; // Ближайшая точка - p1
		}
		if (t > 1.0f)
		{
			return segment_site.p2; // Ближайшая точка - p2
		}
		// Проекция лежит на отрезке
		return segment_site.p1 + ab.Scale(t);
	}
};

struct Edge
{
	Segment segment;
	size_t cell_source_idx = 0;
};

namespace
{
float distance_point_to_feature(const Point& p, const ObstacleFeature& feature)
{
	Point closest_on_feature = feature.getPsi(p);
	return (p - closest_on_feature).Length();
}

std::optional<ObstacleFeature> FindObstacleFeature(
	const boost::polygon::voronoi_cell<double>* cell,
	const std::vector<BoostInputSourceInfo>& source_info_map)
{
	int category = cell->source_category();
	size_t source_idx = cell->source_index();
	std::optional<ObstacleFeature> feature_o_for_psi = std::nullopt;
	const Segment& original_input_segment = source_info_map[source_idx].original_segment;

	constexpr int cat_start_endpoint = SOURCE_CATEGORY_SEGMENT_START_POINT;
	constexpr int cat_end_endpoint = SOURCE_CATEGORY_SEGMENT_END_POINT;

	if (cell->contains_segment())
	{
		feature_o_for_psi = ObstacleFeature(original_input_segment);
	}
	else if (category == cat_start_endpoint)
	{
		feature_o_for_psi = ObstacleFeature(original_input_segment.p2);
	}
	else if (category == cat_end_endpoint)
	{
		feature_o_for_psi = ObstacleFeature(original_input_segment.p1);
	}

	return feature_o_for_psi;
}

void RemoveSegmentsWithPoints(
	std::vector<Segment>& segments,
	const std::set<Point>& pointsToRemove)
{
	auto new_end = std::remove_if(segments.begin(), segments.end(),
		[&](const Segment& seg) {
			return pointsToRemove.contains(seg.p1) || pointsToRemove.contains(seg.p2);
		});

	segments.erase(new_end, segments.end());
}

/**
 * @brief Подготавливает входные данные для библиотеки Boost.Voronoi.
 * @param borderRect Границы области.
 * @param polygons Вектор полигонов-препятствий.
 * @param[out] out_boost_segments Выходной вектор сегментов для Boost.
 * @param[out] out_source_info_map Выходной вектор для сопоставления boost-индексов с исходной геометрией.
 */
void PrepareBoostInputs(
	const BorderRect& borderRect,
	const std::vector<Polygon>& polygons,
	std::vector<BoostSegment>& out_boost_segments,
	std::vector<BoostInputSourceInfo>& out_source_info_map)
{
	// Добавляем рамку
	float left = borderRect.left;
	float top = borderRect.top;
	float right = borderRect.right;
	float bottom = borderRect.bottom;

	auto add_border_segment = [&](Point p1, Point p2) {
		out_boost_segments.emplace_back(point_data(p1.x, p1.y), point_data(p2.x, p2.y));
		out_source_info_map.emplace_back(true, Segment{ p1, p2 }); // is_obstacle_related = false
	};

	add_border_segment({ left, top }, { right, top });
	add_border_segment({ right, top }, { right, bottom });
	add_border_segment({ right, bottom }, { left, bottom });
	add_border_segment({ left, bottom }, { left, top });

	// Добавляем сегменты полигонов-препятствий
	for (const auto& polygon : polygons)
	{
		size_t n = polygon.size();
		for (size_t i = 0; i < n; ++i)
		{
			Point p1_shape = polygon[i];
			Point p2_shape = polygon[(i + 1) % n];
			out_boost_segments.emplace_back(point_data(p1_shape.x, p1_shape.y), point_data(p2_shape.x, p2_shape.y));
			out_source_info_map.emplace_back(true, Segment{ p1_shape, p2_shape }); // is_obstacle_related = true
		}
	}
}

/**
 * @brief Генерирует ребра уточненной диаграммы Вороного (типы 0, i, ii) из построенной диаграммы Boost.
 * @param vd Диаграмма Вороного от Boost.
 * @param source_info_map Карта для получения информации об исходных сайтах.
 * @param polygons Препятствия для проверки валидности ребер.
 * @param num_segments_parabola Количество сегментов для дискретизации парабол.
 * @return Вектор сегментов, составляющих геометрию tilde_V.
 */
std::vector<Edge> GenerateTildeVEdges(
	const voronoi_diagram<double>& vd,
	const std::vector<BoostInputSourceInfo>& source_info_map,
	const std::vector<Polygon>& polygons,
	size_t num_segments_parabola)
{
	std::vector<Edge> generated_edges;
	std::set<Segment> added_edges_tracker;

	auto add_segment_if_valid =
		[&](Point p1, Point p2, size_t idx, Color color = Color::Black) {
			if (p1 == p2)
				return;
			Point mid_vd = p1.Midpoint(p2);
			for (const auto& poly : polygons)
			{
				if (math::Contains(mid_vd, poly))
					return;
			}
			Segment seg = { p1, p2, color };
			if (!added_edges_tracker.contains(seg))
			{
				generated_edges.emplace_back(seg, idx);
				added_edges_tracker.insert(seg);
			}
		};

	// 1. Отфильтрованные стандартные ребра Вороного (Тип 0)
	for (const auto& edge : vd.edges())
	{
		size_t cell_idx = edge.cell()->source_index();
		if (edge.is_infinite())
			continue;
		if (edge.is_curved())
		{
			auto discretized = DiscretizeParabolicVoronoiEdge(edge, source_info_map, num_segments_parabola);
			for (const auto& seg : discretized)
				add_segment_if_valid(seg.p1, seg.p2, cell_idx);
		}
		else
		{
			add_segment_if_valid(
				boost_compat::VertexToPoint(*edge.vertex0()),
				boost_compat::VertexToPoint(*edge.vertex1()),
				cell_idx);
		}
	}

// 2. Ребра типа (i): x * psi_o(x)
#if 0
	for (const auto& boost_v_ref : vd.vertices())
	{
		Point x_vor_vertex(boost_v_ref.x(), boost_v_ref.y());
		const auto* incident_edge = boost_v_ref.incident_edge();
		if (!incident_edge)
			continue;

		std::set<size_t> processed_sites;
		const auto* current_iter_edge = incident_edge;
		do
		{
			if (const auto* cell = current_iter_edge->cell())
			{
				size_t source_idx = cell->source_index();
				int category = cell->source_category();
				size_t site_key = source_idx * 10 + category; // Уникальный ключ для сайта

				if (processed_sites.contains(site_key))
				{
					current_iter_edge = current_iter_edge->rot_next();
					continue;
				}
				if (source_info_map[source_idx].is_obstacle_related)
				{
					std::optional<ObstacleFeature> feature_o = FindObstacleFeature(cell, source_info_map);
					if (feature_o)
					{
						add_segment_if_valid(x_vor_vertex, feature_o->getPsi(x_vor_vertex), source_idx, Color::Blue);
						processed_sites.insert(site_key);
					}
				}
			}
			current_iter_edge = current_iter_edge->rot_next();
		} while (current_iter_edge != incident_edge);
	}
#endif
// 3. Добавление ребер типа (ii): x_min * psi_o(x_min)
#if 0
	for (const auto& cell : vd.cells())
	{
		size_t source_idx = cell.source_index();
		if (source_idx >= source_info_map.size() || !source_info_map[source_idx].is_obstacle_related)
			continue;

		std::optional<ObstacleFeature> feature_o = FindObstacleFeature(&cell, source_info_map);
		if (!feature_o)
			continue;

		auto v_edge = cell.incident_edge();
		if (!v_edge)
			continue;
		auto start_iter_edge = v_edge;
		do
		{
			if (v_edge->is_secondary() || !v_edge->vertex0() || !v_edge->vertex1())
			{
				v_edge = v_edge->next();
				continue;
			}
			Point p_a = boost_compat::VertexToPoint(*v_edge->vertex0());
			Point p_b = boost_compat::VertexToPoint(*v_edge->vertex1());
			Point x_star;
			if (v_edge->is_linear())
			{
				x_star = (distance_point_to_feature(p_a, *feature_o) <= distance_point_to_feature(p_b, *feature_o)) ? p_a : p_b;
			}
			else
			{
				std::vector<Segment> discretized_segments = DiscretizeParabolicVoronoiEdge(
					*v_edge, source_info_map, num_segments_parabola);

				for (const auto& [p1, p2, color] : discretized_segments)
				{
					float dist_pa_to_o = distance_point_to_feature(p1, *feature_o);
					float dist_pb_to_o = distance_point_to_feature(p2, *feature_o);

					if (dist_pa_to_o <= dist_pb_to_o)
					{
						x_star = p_a;
					}
					else
					{
						x_star = p_b;
					}
				}
			}
			add_segment_if_valid(x_star, feature_o->getPsi(x_star), source_idx, Color::Cyan);
			v_edge = v_edge->next();
		} while (v_edge != start_iter_edge);
	}
#endif

	return generated_edges;
}

namespace bg = boost::geometry;

Point ToPoint(const bg::model::d2::point_xy<float>& bg_point)
{
	return { bg::get<0>(bg_point), bg::get<1>(bg_point) };
}

Polygon SegmentsToPolygon(const std::vector<Edge>& segments)
{
	if (segments.empty())
	{
		return {};
	}

	std::set<Point> unique_points;
	for (const auto& edge : segments)
	{
		unique_points.insert(edge.segment.p1);
		unique_points.insert(edge.segment.p2);
	}

	if (unique_points.size() < 1)
	{
		return {};
	}
	else if (unique_points.size() == 1)
	{
		return { *unique_points.begin() };
	}

	typedef bg::model::d2::point_xy<float> bg_point_type;
	typedef bg::model::multi_point<bg_point_type> bg_multi_point_type;
	typedef bg::model::polygon<bg_point_type> bg_polygon_type;

	bg_multi_point_type bg_input_points;

	for (const auto& p : unique_points)
	{
		bg::append(bg_input_points, bg_point_type(p.x, p.y));
	}

	bg_polygon_type bg_hull;
	bg::convex_hull(bg_input_points, bg_hull);

	// For debugging: print the result in WKT
	std::cout << "Extracted unique points (WKT): " << bg::wkt(bg_input_points) << std::endl;
	std::cout << "Boost.Geometry Convex Hull (WKT): " << bg::wkt(bg_hull) << std::endl;

	Polygon result_polygon;
	for (const auto& bg_p : bg::exterior_ring(bg_hull))
	{
		result_polygon.push_back(ToPoint(bg_p));
	}

	if (!result_polygon.empty() && result_polygon.size() > 1 && result_polygon.front() == result_polygon.back())
	{
		result_polygon.pop_back();
	}

	return result_polygon;
}

/**
 * @brief Восстанавливает топологию графа tilde_V, находя все его ячейки (грани).
 * @param tilde_v_edges Вектор сегментов, составляющих геометрию tilde_V.
 * @return Вектор полигонов, каждый из которых представляет ячейку tilde_V.
 */
std::vector<Polygon> ReconstructTildeVTopology(
	const voronoi_diagram<double> vd,
	const std::vector<Edge>& tilde_v_edges)
{
	std::map<Point, std::vector<Point>> adj_list_tilde_V;
	std::set<Point> all_vertices_tilde_V_set; // Используем set для автоматической уникальности и сортировки (если Point::operator< есть)

	std::map<size_t, std::vector<Edge>> all_vertices_tilde_V = GroupBy(tilde_v_edges,
		[](const Edge& edge) { return edge.cell_source_idx; });

	std::vector<Polygon> result;
	result.reserve(all_vertices_tilde_V.size());

	for (const auto& [cell_sourse_idx, edges] : all_vertices_tilde_V)
	{
		result.emplace_back(SegmentsToPolygon(edges));
	}

	return result;

	// for (const auto& [p1, p2] : tilde_v_edges)
	// {
	// 	adj_list_tilde_V[p1].push_back(p2);
	// 	adj_list_tilde_V[p2].push_back(p1);
	// 	all_vertices_tilde_V_set.insert(p1);
	// 	all_vertices_tilde_V_set.insert(p2);
	// }
	//
	// // Шаг 2: Отсортировать списки смежности по полярному углу (против часовой стрелки)
	// for (auto& [u, v] : adj_list_tilde_V)
	// {
	// 	Point center_node = u;
	// 	std::ranges::sort(v,
	// 		[&center_node](const Point& pa, const Point& pb) {
	// 			return std::atan2(pa.y - center_node.y, pa.x - center_node.x) < std::atan2(pb.y - center_node.y, pb.x - center_node.x);
	// 		});
	// }
	//
	// std::vector<Polygon> found_cells_T;
	// std::set<std::pair<Point, Point>> visited_half_edges; // Для отслеживания пройденных полуребер
	//
	// for (const Point& start_node_for_face : all_vertices_tilde_V_set)
	// {
	// 	if (!adj_list_tilde_V.contains(start_node_for_face))
	// 		continue; // Узел может не иметь ребер (изолированный)
	//
	// 	for (const Point& first_neighbor : adj_list_tilde_V.at(start_node_for_face))
	// 	{
	// 		Point p_curr = start_node_for_face;
	// 		Point p_next = first_neighbor;
	// 		std::pair current_half_edge = { p_curr, p_next };
	//
	// 		if (visited_half_edges.contains(current_half_edge))
	// 		{
	// 			continue; // Это полуребро уже было начальной точкой обхода грани
	// 		}
	//
	// 		Polygon current_cell_polygon;
	// 		Point path_tracer_prev = p_curr; // Точка, из которой пришли в p_next
	// 		Point path_tracer_curr = p_next; // Текущая вершина на границе грани
	//
	// 		// Трассировка одной грани
	// 		while (true)
	// 		{
	// 			visited_half_edges.insert({ path_tracer_prev, path_tracer_curr });
	// 			current_cell_polygon.push_back(path_tracer_prev); // Добавляем вершину, из которой исходит ребро грани
	//
	// 			const auto& neighbors_of_tracer_curr = adj_list_tilde_V.at(path_tracer_curr);
	// 			if (neighbors_of_tracer_curr.empty())
	// 				break; // Ошибка или конец графа
	//
	// 			// Ищем ребро (path_tracer_curr, path_tracer_prev) в списке соседей path_tracer_curr
	// 			auto it_incoming_edge_neighbor = std::ranges::find(
	// 				neighbors_of_tracer_curr, path_tracer_prev);
	// 			if (it_incoming_edge_neighbor == neighbors_of_tracer_curr.end())
	// 				break; // Ошибка
	//
	// 			size_t idx_incoming = std::distance(neighbors_of_tracer_curr.begin(), it_incoming_edge_neighbor);
	//
	// 			// Следующее ребро грани в обходе против часовой стрелки - это ребро перед входящим ребром (в отсортированном списке)
	// 			Point next_node_for_face = neighbors_of_tracer_curr[(idx_incoming - 1 + neighbors_of_tracer_curr.size()) % neighbors_of_tracer_curr.size()];
	//
	// 			path_tracer_prev = path_tracer_curr;
	// 			path_tracer_curr = next_node_for_face;
	//
	// 			if (path_tracer_prev == start_node_for_face && path_tracer_curr == first_neighbor)
	// 			{
	// 				break; // Замкнули цикл, вернулись к первому ребру грани
	// 			}
	// 			if (current_cell_polygon.size() > all_vertices_tilde_V_set.size() * 2)
	// 			{
	// 				// Защита от зацикливания
	// 				LOG_WARN("Warning: Face traversal seems to be in an infinite loop.");
	// 				current_cell_polygon.clear(); // Помечаем как невалидную
	// 				break;
	// 			}
	// 		}
	//
	// 		if (!current_cell_polygon.empty() && current_cell_polygon.size() >= 3)
	// 		{
	// 			// Проверка на "внешнюю" грань. Внешняя грань обычно одна.
	// 			// Простой эвристикой может быть проверка площади или ориентации.
	// 			// Для корректной работы Contains() полигон должен быть с правильной ориентацией (например, против ЧС).
	// 			// Алгоритм выше должен давать грани против ЧС, если atan2 отсортировал ребра против ЧС.
	// 			// Пропустим сложную фильтрацию внешней грани пока.
	// 			found_cells_T.push_back(current_cell_polygon);
	// 		}
	// 	}
	// }
	//
	// return found_cells_T;
}

VoronoiData ConstructRefined(
	BorderRect borderRect,
	const std::vector<Polygon>& polygons,
	const Point& start_point_s,
	const Point& target_point_t,
	size_t segments_on_arc)
{
	VoronoiData data;

	std::vector<BoostSegment> input_to_boost_segments;
	std::vector<BoostInputSourceInfo> source_info_map;

	PrepareBoostInputs(borderRect, polygons, input_to_boost_segments, source_info_map);

	voronoi_diagram<double> vd;
	construct_voronoi(input_to_boost_segments.begin(), input_to_boost_segments.end(), &vd);

	auto edges = GenerateTildeVEdges(vd, source_info_map, polygons, segments_on_arc);
	std::vector<Segment> segments;
	for (const auto& edge : edges)
	{
		segments.push_back(edge.segment);
	}

	data.refined_edges = segments;

	data.all_tilde_V_cells = ReconstructTildeVTopology(edges);

	for (size_t i = 0; i < data.all_tilde_V_cells.size(); ++i)
	{
		if (Contains(start_point_s, data.all_tilde_V_cells[i]))
		{
			data.s_cell_idx = i;
		}
		if (Contains(target_point_t, data.all_tilde_V_cells[i]))
		{
			data.t_cell_idx = i;
		}
	}

	if (data.s_cell_idx == -1)
	{
		LOG_CRITICAL(std::format("Failed to locate start point [{}, {}] in any tilde_V cell.", start_point_s.x, start_point_s.y));
	}
	if (data.t_cell_idx == -1)
	{
		LOG_CRITICAL(std::format("Failed to locate target point [{}, {}] in any tilde_V cell.", target_point_t.x, target_point_t.y));
	}

	RemoveSegmentsWithPoints(data.refined_edges,
		{ borderRect.LeftTop(), borderRect.RightBottom(), borderRect.RightTop(), borderRect.LeftBottom() });

	return data;
}

} // namespace

Voronoi::Voronoi(BorderRect borderRect)
	: m_borderRect(borderRect)
{
}

BorderRect Voronoi::GetBorderRect() const
{
	return m_borderRect;
}

void Voronoi::SetBorderRect(BorderRect borderRect)
{
	m_borderRect = borderRect;
}

VoronoiData Voronoi::Generate(
	const std::vector<Polygon>& polygons,
	const Point& start_point_s,
	const Point& target_point_t,
	size_t nums_segment_arc)
{
	m_lastGeneration = ConstructRefined(m_borderRect, polygons, start_point_s, target_point_t, nums_segment_arc);
	return m_lastGeneration;
}

const VoronoiData& Voronoi::GetLastGeneration() const
{
	return m_lastGeneration;
}
