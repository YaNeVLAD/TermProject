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
		out_source_info_map.emplace_back(false, Segment{ p1, p2 }); // is_obstacle_related = false
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

bg::model::d2::point_xy<double> to_bg_point(const Point& p)
{
	return { static_cast<double>(p.x), static_cast<double>(p.y) };
}

// Вспомогательная функция для преобразования boost::geometry point в tp::shapes::Point
Point to_tp_point(const bg::model::d2::point_xy<double>& p)
{
	return { static_cast<float>(bg::get<0>(p)), static_cast<float>(bg::get<1>(p)) };
}

Point get_effective_site_point_for_direction_calc(
	const voronoi_diagram<double>::cell_type* cell,
	const std::vector<BoostInputSourceInfo>& source_info_map)
{

	size_t source_idx = cell->source_index();
	if (source_idx >= source_info_map.size())
	{
		LOG_ERROR(std::format("get_effective_site_point_for_direction_calc: Invalid source_idx {}. Returning (0,0).", source_idx));
		return Point(0, 0);
	}
	int category = cell->source_category();
	const Segment& original_input_segment = source_info_map[source_idx].original_segment;

	if (category == SOURCE_CATEGORY_SEGMENT_START_POINT || category == SOURCE_CATEGORY_SINGLE_POINT)
	{
		return original_input_segment.p1;
	}
	else if (category == SOURCE_CATEGORY_SEGMENT_END_POINT)
	{
		return original_input_segment.p2;
	}
	else if (cell->contains_segment())
	{
		LOG_WARN(std::format("get_effective_site_point_for_direction_calc: Linear infinite edge generated by full segment (source_idx {}). Using p1 heuristic.",
			source_idx));
		return original_input_segment.p1;
	}

	LOG_ERROR(std::format("get_effective_site_point_for_direction_calc: Unknown source category {} for cell source_idx {}. Returning (0,0).",
		category, source_idx));
	return Point(0, 0); // Fallback
}

/**
 * @brief Реализует обрезку бесконечного ребра диаграммы Вороного по ограничивающей рамке.
 * Использует Boost.Geometry для вычислений пересечений.
 * @param edge Указатель на бесконечное ребро Boost.Polygon.
 * @param clipped_points Вектор, куда будут добавлены точки обрезанного сегмента.
 * @param clipping_box Ограничивающая рамка для обрезки.
 * @param source_info_map Карта информации об исходных сайтах (НЕОБХОДИМА для определения направления).
 * @return true, если обрезка прошла успешно и точки добавлены; false в противном случае.
 */
bool clip_infinite_voronoi_edge(
	const voronoi_diagram<double>::edge_type* edge,
	std::vector<Point>& clipped_points,
	const bg::model::box<bg::model::d2::point_xy<double>>& clipping_box,
	const std::vector<BoostInputSourceInfo>& source_info_map) // Добавленный аргумент
{
	return false;

	Point finite_voronoi_vertex;
	if (edge->vertex0())
	{
		finite_voronoi_vertex = boost_compat::VertexToPoint(*edge->vertex0());
	}
	else if (edge->vertex1())
	{
		finite_voronoi_vertex = boost_compat::VertexToPoint(*edge->vertex1());
	}
	else
	{
		LOG_ERROR("clip_infinite_voronoi_edge: Infinite edge has no finite vertex.");
		return false;
	}

	Point site_point1 = get_effective_site_point_for_direction_calc(edge->cell(), source_info_map);
	Point site_point2 = get_effective_site_point_for_direction_calc(edge->twin()->cell(), source_info_map);

	if (site_point1 == site_point2)
	{
		LOG_WARN(std::format("clip_infinite_voronoi_edge: Effective site points for edge are identical. Cannot determine direction."));
		return false;
	}

	Vec2D vec_sites = site_point2 - site_point1;
	Vec2D normal1 = Vec2D(-vec_sites.y, vec_sites.x);
	Vec2D normal2 = Vec2D(vec_sites.y, -vec_sites.x);

	Vec2D ray_direction;
	const double test_step = 1.0;
	Point test_point1 = finite_voronoi_vertex + normal1.Normalize() * test_step;
	Point test_point2 = finite_voronoi_vertex + normal2.Normalize() * test_step;

	double dist1_to_s1 = (test_point1 - site_point1).LengthSquared();
	double dist1_to_s2 = (test_point1 - site_point2).LengthSquared();
	double dist2_to_s1 = (test_point2 - site_point1).LengthSquared();
	double dist2_to_s2 = (test_point2 - site_point2).LengthSquared();

	if (std::abs(dist1_to_s1 - dist1_to_s2) < std::abs(dist2_to_s1 - dist2_to_s2))
	{
		ray_direction = normal1;
	}
	else
	{
		ray_direction = normal2;
	}

	ray_direction = ray_direction.Normalize();

	const double large_value = 100000.0;
	Point far_point = finite_voronoi_vertex + ray_direction * large_value;

	bg::model::segment<bg::model::d2::point_xy<double>> bg_ray_segment(
		to_bg_point(finite_voronoi_vertex), to_bg_point(far_point));

	std::vector<bg::model::linestring<bg::model::d2::point_xy<double>>> intersections_linestrings;
	bg::intersection(clipping_box, bg_ray_segment, intersections_linestrings);

	if (intersections_linestrings.empty())
	{
		LOG_WARN(std::format("clip_infinite_voronoi_edge: Ray from vertex [{},{}] does not intersect clipping box.",
			finite_voronoi_vertex.x, finite_voronoi_vertex.y));
		return false;
	}

	if (intersections_linestrings.size() == 1)
	{
		const auto& clipped_linestring = intersections_linestrings[0];
		if (clipped_linestring.size() >= 2)
		{
			clipped_points.push_back(to_tp_point(clipped_linestring.front()));
			clipped_points.push_back(to_tp_point(clipped_linestring.back()));
		}
		else
		{
			LOG_WARN("clip_infinite_voronoi_edge: Clipped linestring has less than 2 points.");
			return false;
		}
	}
	else
	{
		const auto& first_linestring = intersections_linestrings[0];
		if (first_linestring.size() >= 2)
		{
			clipped_points.push_back(to_tp_point(first_linestring.front()));
			clipped_points.push_back(to_tp_point(first_linestring.back()));
		}
		else
		{
			LOG_WARN("clip_infinite_voronoi_edge: First clipped linestring has less than 2 points.");
			return false;
		}
	}
	if (!clipped_points.empty() && clipped_points.front() != finite_voronoi_vertex)
	{
		if (bg::within(to_bg_point(finite_voronoi_vertex), clipping_box))
		{
			clipped_points.insert(clipped_points.begin(), finite_voronoi_vertex);
		}
	}

	if (clipped_points.size() < 2)
	{
		LOG_WARN("clip_infinite_voronoi_edge: Failed to construct a valid clipped segment (less than 2 points).");
		return false;
	}

	if (bg::within(to_bg_point(finite_voronoi_vertex), clipping_box) && clipped_points.front() != finite_voronoi_vertex)
	{
		clipped_points.insert(clipped_points.begin(), finite_voronoi_vertex);
	}
	if (clipped_points.size() > 2)
	{
		Point p_start = clipped_points.front();
		Point p_end = clipped_points.back();
		clipped_points.clear();
		clipped_points.push_back(p_start);
		clipped_points.push_back(p_end);
	}

	return true;
}

/**
 * @brief Восстанавливает топологию графа tilde_V, находя все его ячейки (грани).
 * @param tilde_v_edges Вектор сегментов, составляющих геометрию tilde_V.
 * @return Вектор полигонов, каждый из которых представляет ячейку tilde_V.
 */
std::vector<Polygon> ReconstructTildeVTopology(
	const voronoi_diagram<double>& vd,
	const std::vector<BoostInputSourceInfo>& source_info_map,
	const bg::model::box<bg::model::d2::point_xy<double>>& clipping_box,
	size_t num_segments_parabola)
{
	std::vector<Polygon> result_polygons;

	// Итерируем по всем ячейкам в исходной диаграмме Вороного Boost.Polygon
	for (auto it = vd.cells().begin();
		it != vd.cells().end(); ++it)
	{
		const auto& cell = *it;

		const auto* edge = cell.incident_edge();
		if (!edge)
			continue; // Пустая ячейка или некорректная

		std::vector<Point> current_cell_points;
		std::vector<Point> temp_edge_points;

		// Обход рёбер вокруг ячейки в counter-clockwise (CCW) порядке
		do
		{
			temp_edge_points.clear();

			if (edge->is_linear())
			{
				if (edge->is_finite())
				{
					// Конечное линейное ребро
					if (edge->vertex0() && edge->vertex1())
					{
						temp_edge_points.push_back(boost_compat::VertexToPoint(*edge->vertex0()));
						temp_edge_points.push_back(boost_compat::VertexToPoint(*edge->vertex1()));
					}
				}
				else
				{
					// Бесконечное линейное ребро - требуется обрезка
					if (!clip_infinite_voronoi_edge(edge, temp_edge_points, clipping_box, source_info_map))
					{
						LOG_ERROR(std::format("Failed to clip infinite edge for cell source_idx {}.", cell.source_index()));
						// Если обрезка не удалась, возможно, лучше пропустить эту ячейку
						// или оставить её открытой, в зависимости от требований.
						// Для получения замкнутых полигонов, это критично.
						current_cell_points.clear(); // Очищаем, чтобы не формировать неполный полигон
						break; // Прерываем обход для этой ячейки
					}
				}
			}
			else if (edge->is_curved())
			{
				// Параболическое ребро - требуется дискретизация
				auto temp = DiscretizeParabolicVoronoiEdge(
					*edge, source_info_map, num_segments_parabola);

				for (const auto& seg : temp)
				{
					temp_edge_points.push_back(seg.p1);
					temp_edge_points.push_back(seg.p2);
				}
			}

			// Добавляем точки ребра в список точек текущей ячейки
			if (!temp_edge_points.empty())
			{
				if (current_cell_points.empty())
				{
					current_cell_points.insert(current_cell_points.end(), temp_edge_points.begin(), temp_edge_points.end());
				}
				else
				{
					// Избегаем дублирования первой точки, если она совпадает с последней
					// Это важно для правильного формирования контура полигона
					if (current_cell_points.back() == temp_edge_points.front())
					{
						current_cell_points.insert(current_cell_points.end(), temp_edge_points.begin() + 1, temp_edge_points.end());
					}
					else
					{
						current_cell_points.insert(current_cell_points.end(), temp_edge_points.begin(), temp_edge_points.end());
					}
				}
			}

			edge = edge->next();
		} while (edge != cell.incident_edge());

		// После сбора всех точек, формируем Polygon
		if (current_cell_points.size() >= 3)
		{ // Полигон должен иметь минимум 3 точки
			// Проверяем и замыкаем полигон, если первая и последняя точки не совпадают
			if (!current_cell_points.empty() && current_cell_points.front() != current_cell_points.back())
			{
				current_cell_points.push_back(current_cell_points.front());
			}

			result_polygons.push_back(Polygon(current_cell_points));
		}
	}
	return result_polygons;
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

	bg::model::box<bg::model::d2::point_xy<double>> clipping_box(
		bg::model::d2::point_xy<double>(borderRect.left, borderRect.bottom),
		bg::model::d2::point_xy<double>(borderRect.right, borderRect.top));

	auto edges = GenerateTildeVEdges(vd, source_info_map, polygons, segments_on_arc);
	std::vector<Segment> segments;
	for (const auto& edge : edges)
	{
		segments.push_back(edge.segment);
	}

	data.refined_edges = segments;

	data.all_tilde_V_cells = ReconstructTildeVTopology(vd, source_info_map, clipping_box, segments_on_arc);

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

	// RemoveSegmentsWithPoints(data.refined_edges,
	//{ borderRect.LeftTop(), borderRect.RightBottom(), borderRect.RightTop(), borderRect.LeftBottom() });

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
