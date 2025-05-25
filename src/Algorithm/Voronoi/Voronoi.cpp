//
// Created by User on 19.04.2025.
//

#include "Voronoi.h"

#include <boost/polygon/voronoi.hpp>

#include "../../Utility/Logger/Logger.h"
#include "../Structures/BoostCompat.h"

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

struct BoostInputSourceInfo
{
	bool is_obstacle_related;
	Segment original_segment;

	explicit BoostInputSourceInfo(bool is_obs, Segment seg = {})
		: is_obstacle_related(is_obs)
		, original_segment(seg)
	{
	}
};

namespace
{
// DISCRETE EDGE
Point ProjectPointToLine(const Point& p, const Point& l_p1, const Point& l_p2)
{
	Vec2D line_vec = l_p2 - l_p1;
	if (line_vec.LengthSquared() < FLT_EPSILON * FLT_EPSILON)
	{ // Прямая вырождена в точку
		return l_p1;
	}
	Vec2D p_to_l_p1_vec = p - l_p1;
	float t = (p_to_l_p1_vec.x * line_vec.x + p_to_l_p1_vec.y * line_vec.y) / line_vec.LengthSquared();
	return l_p1 + line_vec.Scale(t);
}

std::optional<Point> IntersectLines(const Point& p1, const Point& p2, const Point& p3, const Point& p4)
{
	float den = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);
	if (std::abs(den) < FLT_EPSILON)
	{ // Знаменатель близок к нулю
		return std::nullopt;
	}

	float t_num = (p1.x - p3.x) * (p3.y - p4.y) - (p1.y - p3.y) * (p3.x - p4.x);
	// float u_num = -((p1.x - p2.x) * (p1.y - p3.y) - (p1.y - p2.y) * (p1.x - p3.x)); // Для второй прямой

	float t = t_num / den;
	// float u = u_num / den; // u можно использовать для проверки, лежит ли пересечение на отрезке p3p4

	return Point{ p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y) };
}

void DiscretizeRecursive(
	const Point& focus,
	const Point& directrix_l_p1, const Point& directrix_l_p2,
	const Point& arc_p0, const Point& arc_p2,
	size_t current_depth,
	std::vector<Point>& points_on_arc_accumulator) // Сюда добавляются точки (arc_p0 уже должна быть там)
{
	if (current_depth == 0 || arc_p0 == arc_p2)
	{ // Базовый случай рекурсии или вырожденная дуга
		if (points_on_arc_accumulator.empty() || points_on_arc_accumulator.back() != arc_p2)
		{ // Проверка на дубликат
			if (arc_p0 != arc_p2 && !points_on_arc_accumulator.empty() && points_on_arc_accumulator.back() == arc_p0)
			{
				points_on_arc_accumulator.push_back(arc_p2);
			}
		}
		return;
	}

	// Касательная в arc_p0
	Point m0 = ProjectPointToLine(arc_p0, directrix_l_p1, directrix_l_p2);
	Vec2D v_p0_f = focus - arc_p0;
	Vec2D v_p0_m0 = m0 - arc_p0; // Вектор от точки на параболе к ее проекции на директрису

	// Проверка на вырожденные случаи (точка на фокусе или директрисе, коллинеарность)
	if (v_p0_f.LengthSquared() < FLT_EPSILON * FLT_EPSILON || v_p0_m0.LengthSquared() < FLT_EPSILON * FLT_EPSILON)
	{
		LOG_DEBUG("Degenerate case for tangent at P0 during parabola discretization.");
		// Простой fallback: делим хорду пополам
		Point mid_chord = arc_p0.Midpoint(arc_p2);
		DiscretizeRecursive(focus, directrix_l_p1, directrix_l_p2, arc_p0, mid_chord, current_depth - 1, points_on_arc_accumulator);
		DiscretizeRecursive(focus, directrix_l_p1, directrix_l_p2, mid_chord, arc_p2, current_depth - 1, points_on_arc_accumulator);
		return;
	}
	Vec2D dir_tangent_p0 = v_p0_f.Normalize() + v_p0_m0.Normalize();
	if (dir_tangent_p0.LengthSquared() < FLT_EPSILON * FLT_EPSILON)
	{ // Направления противоположны (F-P0-M0 коллинеарны)
		LOG_DEBUG("Zero tangent direction at P0.");
		Point mid_chord = arc_p0.Midpoint(arc_p2); // Fallback
		DiscretizeRecursive(focus, directrix_l_p1, directrix_l_p2, arc_p0, mid_chord, current_depth - 1, points_on_arc_accumulator);
		DiscretizeRecursive(focus, directrix_l_p1, directrix_l_p2, mid_chord, arc_p2, current_depth - 1, points_on_arc_accumulator);
		return;
	}
	dir_tangent_p0 = dir_tangent_p0.Normalize();
	Point t0_plus_dir = arc_p0 + dir_tangent_p0; // Вторая точка для определения прямой-касательной T0

	// Касательная в arc_p2
	Point m2 = ProjectPointToLine(arc_p2, directrix_l_p1, directrix_l_p2);
	Vec2D v_p2_f = focus - arc_p2;
	Vec2D v_p2_m2 = m2 - arc_p2;
	if (v_p2_f.LengthSquared() < FLT_EPSILON * FLT_EPSILON || v_p2_m2.LengthSquared() < FLT_EPSILON * FLT_EPSILON)
	{
		LOG_DEBUG("Degenerate case for tangent at P2 during parabola discretization.");
		Point mid_chord = arc_p0.Midpoint(arc_p2); // Fallback
		DiscretizeRecursive(focus, directrix_l_p1, directrix_l_p2, arc_p0, mid_chord, current_depth - 1, points_on_arc_accumulator);
		DiscretizeRecursive(focus, directrix_l_p1, directrix_l_p2, mid_chord, arc_p2, current_depth - 1, points_on_arc_accumulator);
		return;
	}
	Vec2D dir_tangent_p2 = v_p2_f.Normalize() + v_p2_m2.Normalize();
	if (dir_tangent_p2.LengthSquared() < FLT_EPSILON * FLT_EPSILON)
	{
		LOG_DEBUG("Zero tangent direction at P2.");
		Point mid_chord = arc_p0.Midpoint(arc_p2); // Fallback
		DiscretizeRecursive(focus, directrix_l_p1, directrix_l_p2, arc_p0, mid_chord, current_depth - 1, points_on_arc_accumulator);
		DiscretizeRecursive(focus, directrix_l_p1, directrix_l_p2, mid_chord, arc_p2, current_depth - 1, points_on_arc_accumulator);
		return;
	}
	dir_tangent_p2 = dir_tangent_p2.Normalize();
	Point t2_plus_dir = arc_p2 + dir_tangent_p2; // Вторая точка для прямой-касательной T2

	std::optional<Point> p1_bezier_control = IntersectLines(arc_p0, t0_plus_dir, arc_p2, t2_plus_dir);

	if (!p1_bezier_control)
	{ // Касательные параллельны или ошибка пересечения
		LOG_DEBUG("Tangents are parallel or intersection failed.");
		// Fallback: делим хорду. Для большей точности можно было бы проецировать середину хорды на параболу,
		// но это сложнее. Пока простой fallback.
		Point mid_chord = arc_p0.Midpoint(arc_p2);
		DiscretizeRecursive(focus, directrix_l_p1, directrix_l_p2, arc_p0, mid_chord, current_depth - 1, points_on_arc_accumulator);
		DiscretizeRecursive(focus, directrix_l_p1, directrix_l_p2, mid_chord, arc_p2, current_depth - 1, points_on_arc_accumulator);
		return;
	}
	Point p1_b = p1_bezier_control.value();

	// Точка на параболе при t=0.5 (середина дуги Безье)
	Vec2D arc_mid_vec = static_cast<Vec2D>(arc_p0).Scale(0.25f)
		+ static_cast<Vec2D>(p1_b).Scale(0.5f)
		+ static_cast<Vec2D>(arc_p2).Scale(0.25f);

	Point arc_mid_point = { arc_mid_vec.x, arc_mid_vec.y };

	DiscretizeRecursive(focus, directrix_l_p1, directrix_l_p2, arc_p0, arc_mid_point, current_depth - 1, points_on_arc_accumulator);
	DiscretizeRecursive(focus, directrix_l_p1, directrix_l_p2, arc_mid_point, arc_p2, current_depth - 1, points_on_arc_accumulator);
}

// VORONOI
bool Contains(const Point& point, const Polygon& polygon)
{
	namespace bg = boost::geometry;
	namespace bgm = boost::geometry::model;

	using boost_point_type = bgm::d2::point_xy<double>;
	using boost_polygon_type = bgm::polygon<boost_point_type>;

	boost_point_type bg_point(point.x, point.y);
	boost_polygon_type bg_polygon;

	for (const auto& [x, y] : polygon)
	{
		bg::append(bg_polygon.outer(), boost_point_type(x, y));
	}

	return bg::within(bg_point, bg_polygon);
}
using namespace boost::polygon;
std::vector<Segment> DiscretizeParabolicVoronoiEdge(
	const voronoi_edge<double>& curved_boost_edge,
	const std::vector<BoostInputSourceInfo>& source_info_map,
	size_t num_segments_hint)
{
	std::vector<Segment> resulting_segments;

	if (curved_boost_edge.is_linear() || !curved_boost_edge.vertex0() || !curved_boost_edge.vertex1())
	{
		if (curved_boost_edge.vertex0() && curved_boost_edge.vertex1())
		{ // Если это просто прямое ребро
			Point u = boost_compat::VertexToPoint(*curved_boost_edge.vertex0());
			Point v = boost_compat::VertexToPoint(*curved_boost_edge.vertex1());
			resulting_segments.emplace_back(u, v);
		}
		return resulting_segments;
	}

	Point p_arc_start = boost_compat::VertexToPoint(*curved_boost_edge.vertex0());
	Point p_arc_end = boost_compat::VertexToPoint(*curved_boost_edge.vertex1());

	if (p_arc_start == p_arc_end)
	{ // Вырожденная дуга
		return resulting_segments; // Пустой вектор
	}

	// Определяем фокус (точечный сайт) и директрису (прямая сегментного сайта)
	Point focus_F;
	Segment directrix_segment_S; // Отрезок, задающий прямую-директрису
	bool focus_identified = false;
	bool directrix_identified = false;

	// Сайты, определяющие ребро
	auto cell1 = curved_boost_edge.cell();
	auto cell2 = curved_boost_edge.twin()->cell();

	std::array<decltype(cell1), 2> cells_for_sites = { cell1, cell2 };
	for (const auto* current_b_cell : cells_for_sites)
	{
		if (!current_b_cell)
			continue;

		size_t src_idx = current_b_cell->source_index();
		if (src_idx >= source_info_map.size())
		{
			LOG_ERROR(std::format("Discretize: source_idx {} out of bounds for source_info_map (size {}).", src_idx, source_info_map.size()));
			continue;
		}

		int category = current_b_cell->source_category();
		const Segment& original_input_for_site = source_info_map[src_idx].original_segment;

		if (current_b_cell->contains_segment())
		{
			if (!directrix_identified)
			{
				directrix_segment_S = original_input_for_site;
				directrix_identified = true;
			}
			else
			{
				// Эта ситуация (оба сайта - сегменты) не должна приводить к параболическому ребру.
				LOG_ERROR("Discretize: Both cells indicate segment sites for a curved edge. This should not happen.");
			}
		}
		else
		{ // current_cell_is_segment_site == false, значит сайт - точка (фокус)
			if (!focus_identified)
			{
				if (category == SOURCE_CATEGORY_SEGMENT_START_POINT)
				{
					focus_F = original_input_for_site.p2;
					focus_identified = true;
				}
				else if (category == SOURCE_CATEGORY_SEGMENT_END_POINT)
				{
					focus_F = original_input_for_site.p1;
					focus_identified = true;
				}
				else
				{
					LOG_WARN(std::format("Discretize: Unhandled point site category {} for cell of curved edge. Original input segment: [{},{}]-[{},{}].",
						category, original_input_for_site.p1.x, original_input_for_site.p1.y,
						original_input_for_site.p2.x, original_input_for_site.p2.y));
				}
			}
			else
			{
				// Оба сайта - точки, это должно давать прямое ребро Вороного (биссектрису отрезка).
				LOG_ERROR("Discretize: Both cells indicate point sites for a curved edge. This should not happen.");
			}
		}
	}

	if (!focus_identified || !directrix_identified)
	{
		LOG_WARN(std::format("Discretize: Could not identify focus and/or directrix for parabolic edge between [{},{}] and [{},{}]. Returning chord.",
			p_arc_start.x, p_arc_start.y, p_arc_end.x, p_arc_end.y));
		resulting_segments.push_back({ p_arc_start, p_arc_end });
		return resulting_segments;
	}

	// Проверка на вырожденный случай: фокус на директрисе
	Point proj_F_on_L = ProjectPointToLine(focus_F, directrix_segment_S.p1, directrix_segment_S.p2);
	if ((proj_F_on_L - focus_F).LengthSquared() < FLT_EPSILON * FLT_EPSILON)
	{
		LOG_WARN(std::format("Discretize: Focus [{},{}] lies on the directrix defined by [{},{}]-[{},{}]. Parabola is degenerate. Returning chord.",
			focus_F.x, focus_F.y, directrix_segment_S.p1.x, directrix_segment_S.p1.y, directrix_segment_S.p2.x, directrix_segment_S.p2.y));
		resulting_segments.push_back({ p_arc_start, p_arc_end });
		return resulting_segments;
	}

	std::vector<Point> points_on_arc_result;
	points_on_arc_result.push_back(p_arc_start); // Начинаем с первой точки

	size_t depth = 0;
	if (num_segments_hint > 1)
	{
		// Глубина рекурсии определяет количество точек $2^{depth}$.
		// Если нужно num_segments_hint отрезков, нужно num_segments_hint+1 точек.
		// depth = ceil(log2(num_segments_hint))
		depth = static_cast<size_t>(std::ceil(std::log2(static_cast<double>(num_segments_hint))));
	}
	else if (num_segments_hint == 1)
	{
		depth = 0; // Только хорда (p_arc_start, p_arc_end)
	}
	else
	{ // Некорректный или нулевой hint
		depth = 3; // Глубина по умолчанию (даст 2^3 = 8 сегментов)
	}

	if (depth > 0)
	{ // Только если нужна рекурсия
		DiscretizeRecursive(focus_F, directrix_segment_S.p1, directrix_segment_S.p2,
			p_arc_start, p_arc_end, depth, points_on_arc_result);
	}
	else
	{ // depth = 0, значит только конечная точка
		if (p_arc_start != p_arc_end)
			points_on_arc_result.push_back(p_arc_end);
	}

	// Преобразуем набор точек в набор сегментов
	// std::sort(points_on_arc_result.begin(), points_on_arc_result.end());
	for (size_t i = 0; i < points_on_arc_result.size() - 1; ++i)
	{
		if (points_on_arc_result[i] == points_on_arc_result[i + 1])
			continue; // Пропуск сегментов нулевой длины
		resulting_segments.emplace_back(points_on_arc_result[i], points_on_arc_result[i + 1]);
	}

	// Если после дискретизации ничего не получилось, а точки разные - добавляем хорду
	if (resulting_segments.empty() && !(p_arc_start == p_arc_end))
	{
		resulting_segments.emplace_back(p_arc_start, p_arc_end);
	}

	return resulting_segments;
}

float distance_point_to_feature(const Point& p, const ObstacleFeature& feature)
{
	Point closest_on_feature = feature.getPsi(p);
	return (p - closest_on_feature).Length();
}

VoronoiData ConstructRefined(
	BorderRect borderRect,
	const std::vector<Polygon>& polygons,
	const Point& start_point_s,
	const Point& target_point_t)
{
	VoronoiData data;

	using namespace boost::polygon;
	using namespace tp::shapes;

	std::vector<BoostSegment> input_to_boost_segments;
	std::vector<BoostInputSourceInfo> source_info_map;

	// Добавляем рамку
	float left = borderRect.left;
	float top = borderRect.top;
	float right = borderRect.right;
	float bottom = borderRect.bottom;

	auto add_border_segment = [&](Point p1, Point p2) {
		input_to_boost_segments.emplace_back(point_data(p1.x, p1.y), point_data(p2.x, p2.y));
		source_info_map.emplace_back(false, Segment{ p1, p2 }); // is_obstacle_related = false
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
			input_to_boost_segments.emplace_back(point_data(p1_shape.x, p1_shape.y), point_data(p2_shape.x, p2_shape.y));
			source_info_map.emplace_back(true, Segment{ p1_shape, p2_shape }); // is_obstacle_related = true
		}
	}

	voronoi_diagram<double> vd;
	construct_voronoi(input_to_boost_segments.begin(), input_to_boost_segments.end(), &vd);

	// Используем std::set для хранения пар точек, чтобы избежать дубликатов ребер.
	// Требует Point::operator< для упорядочивания.
	std::set<Segment> added_edges_tracker;

	auto add_segment_if_valid =
		[&](Point p1, Point p2, const std::vector<Polygon>& current_polygons) {
			if (p1 == p2)
				return;

			// bool is_inside_obstacle = false;
			// Point mid_vd = p1.Midpoint(p2);
			// for (const auto& poly : polygons)
			//{
			//	if (Contains(mid_vd, poly))
			//	{
			//		is_inside_obstacle = true;
			//		break;
			//	}
			// }
			// if (is_inside_obstacle)
			//{
			//	return; // Пропускаем этот сегмент
			// }
			Segment seg = { p1, p2 };
			if (!added_edges_tracker.contains(seg))
			{
				data.refined_edges.push_back(seg);
				added_edges_tracker.insert(seg);
			}
		};

	// 1. Добавляем отфильтрованные стандартные ребра Вороного
	for (const auto& edge : vd.edges())
	{
		if (edge.is_infinite())
			continue;

		if (edge.is_curved())
		{
			size_t desired_segments_for_parabola = 2; // Сделайте это настраиваемым параметром
			std::vector<Segment> discretized_segments = DiscretizeParabolicVoronoiEdge(
				edge, source_info_map, desired_segments_for_parabola);

			for (const auto& [p1, p2] : discretized_segments)
			{
				add_segment_if_valid(p1, p2, polygons);
			}
		}
		else
		{
			Point start_vd = boost_compat::VertexToPoint(*edge.vertex0());
			Point end_vd = boost_compat::VertexToPoint(*edge.vertex1());

			add_segment_if_valid(start_vd, end_vd, polygons);
		}
	}

	// 2. Добавление ребер типа (i): x * \psi_o(x)
	// Для каждой вершины диаграммы Вороного...
	for (const auto& boost_v_ref : vd.vertices())
	{
		// vd.vertices() возвращает коллекцию vertex_type объектов
		Point x_vor_vertex(static_cast<float>(boost_v_ref.x()), static_cast<float>(boost_v_ref.y()));
		const voronoi_edge<double>* incident_edge = boost_v_ref.incident_edge();
		if (!incident_edge)
			continue;

		std::set<size_t> // Уникальные source_indices, уже обработанные для этой вершины VD
			processed_true_sites_for_this_vd_vertex; // Чтобы избежать дублирования из-за twin()

		const voronoi_edge<double>* current_iter_edge = incident_edge;
		do
		{
			if (const auto* cell = current_iter_edge->cell())
			{
				size_t source_idx = cell->source_index();

				// Проверяем, не обработали ли мы уже этот сайт для данной вершины VD
				// (один сайт может быть доступен через несколько полуребер вокруг вершины)
				// Для большей надежности, можно было бы хранить пару (source_idx, category) или хэш от ObstacleFeature
				if (processed_true_sites_for_this_vd_vertex.contains(source_idx * 10 + cell->source_category()))
				{
					current_iter_edge = current_iter_edge->rot_next();
					continue;
				}

				if (source_info_map[source_idx].is_obstacle_related)
				{
					std::optional<ObstacleFeature> feature_o_for_psi = std::nullopt; // Особенность 'o', для которой считаем psi_o(x)

					int category = cell->source_category();
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

					if (feature_o_for_psi)
					{
						add_segment_if_valid(x_vor_vertex, feature_o_for_psi.value().getPsi(x_vor_vertex), polygons);
						processed_true_sites_for_this_vd_vertex.insert(source_idx * 10 + category);
					}
				}
			}
			current_iter_edge = current_iter_edge->rot_next();
		} while (current_iter_edge != incident_edge);
	}

	// 3. Добавление ребер типа (ii): x_min * \psi_o(x_min)
	//    x_min - это точка на ребре Вороного 'e' (из V(o)), которая минимизирует расстояние до 'o'.
	//    Итерируем по ячейкам, затем по их первичным ребрам.
	for (const auto& cell : vd.cells())
	{
		size_t source_idx = cell.source_index();

		// Пропускаем ячейки, не связанные с препятствиями
		if (source_idx >= source_info_map.size() || !source_info_map[source_idx].is_obstacle_related)
		{
			continue;
		}

		// Определяем особенность 'o' для текущей ячейки cell
		std::optional<ObstacleFeature> feature_o_cell = std::nullopt;
		const Segment& original_input_segment_for_cell = source_info_map[source_idx].original_segment;

		constexpr int cat_start_endpoint = SOURCE_CATEGORY_SEGMENT_START_POINT; // или аналогичный
		constexpr int cat_end_endpoint = SOURCE_CATEGORY_SEGMENT_END_POINT; // или аналогичный

		int cell_category = cell.source_category();

		if (cell.contains_segment())
		{
			feature_o_cell = ObstacleFeature(original_input_segment_for_cell);
		}
		else if (cell_category == cat_start_endpoint)
		{
			feature_o_cell = ObstacleFeature(original_input_segment_for_cell.p2);
		}
		else if (cell_category == cat_end_endpoint)
		{
			feature_o_cell = ObstacleFeature(original_input_segment_for_cell.p1);
		}

		if (!feature_o_cell)
		{
			continue;
		}

		// Итерируем по первичным инцидентным ребрам этой ячейки 'V(o)'
		auto v_edge = cell.incident_edge();
		if (!v_edge)
			continue;

		auto start_iter_edge = v_edge;
		do
		{
			// Обрабатываем только первичные, конечные ребра.
			// is_secondary() может помочь отфильтровать внутренние ребра для отрезков-сайтов, если они есть.
			if (v_edge->is_secondary() || !v_edge->vertex0() || !v_edge->vertex1())
			{
				v_edge = v_edge->next();
				continue;
			}
			v_edge->is_curved();

			Point p_a = boost_compat::VertexToPoint(*v_edge->vertex0());
			Point p_b = boost_compat::VertexToPoint(*v_edge->vertex1());

			Point x_star; // Точка на ребре (p_a, p_b), ближайшая к feature_o_cell

			if (v_edge->is_linear())
			{
				// Используем свойство выпуклости: минимум на концах для линейного ребра.
				float dist_pa_to_o = distance_point_to_feature(p_a, *feature_o_cell);
				float dist_pb_to_o = distance_point_to_feature(p_b, *feature_o_cell);

				// Выбираем ту конечную точку ребра Вороного, которая ближе к особенности o.
				// Добавляем небольшое смещение к одному из расстояний для детерминированного выбора при равенстве,
				// хотя это обычно не критично.
				if (dist_pa_to_o <= dist_pb_to_o)
				{
					x_star = p_a;
				}
				else
				{
					x_star = p_b;
				}
			}
			else
			{
				// Криволинейное ребро (парабола)
				// Для параболических ребер свойство выпуклости также сохраняется.
				// Минимум также будет на одном из концов хорды, которую представляет v_edge->vertex0/1(),
				// или в точке на параболе, где ее касательная параллельна "эффективной прямой" особенности 'o',
				// или в точке, где кривизна параболы "смотрит" на 'o'.
				// Это сложно. Самое простое приближение - использовать ту же логику, что и для линейных ребер,
				// т.е. проверить только конечные точки дискретизированного сегмента (хорды).
				// Более точный метод потребовал бы итерации по дискретизированным точкам параболы,
				// которые Boost использует для отрисовки, или аналитического решения.
				// Пока используем ту же эвристику конечных точек.

				size_t desired_segments_for_parabola = 2; // Сделайте это настраиваемым параметром
				std::vector<Segment> discretized_segments = DiscretizeParabolicVoronoiEdge(
					*v_edge, source_info_map, desired_segments_for_parabola);

				for (const auto& [p1, p2] : discretized_segments)
				{
					float dist_pa_to_o = distance_point_to_feature(p1, *feature_o_cell);
					float dist_pb_to_o = distance_point_to_feature(p2, *feature_o_cell);

					if (dist_pa_to_o <= dist_pb_to_o)
					{
						x_star = p_a;
					}
					else
					{
						x_star = p_b;
					}
				}

				// float dist_pa_to_o = distance_point_to_feature(p_a, *feature_o_cell);
				// float dist_pb_to_o = distance_point_to_feature(p_b, *feature_o_cell);

				// if (dist_pa_to_o <= dist_pb_to_o)
				//{
				//	x_star = p_a;
				// }
				// else
				//{
				//	x_star = p_b;
				// }
				//  Примечание: для более точного результата на кривых ребрах, если Boost предоставляет
				//  точки дискретизации для v_edge (например, через какой-либо метод вроде v_edge->discretize(...)),
				//  следовало бы итерировать по этим точкам и найти среди них ту, что минимизирует расстояние до feature_o_cell.
			}

			Point psi_o_x_star = feature_o_cell->getPsi(x_star);
			add_segment_if_valid(x_star, psi_o_x_star, polygons);

			v_edge = v_edge->next();
		} while (v_edge != start_iter_edge);
	}

	// --- НОВЫЙ КОД: Построение графа $\tilde{\mathcal{V}}$ и поиск ячеек для s и t ---

	// Шаг 1: Собрать все уникальные вершины $\tilde{\mathcal{V}}$ и построить список смежности
	std::map<Point, std::vector<Point>> adj_list_tilde_V;
	std::set<Point> all_vertices_tilde_V_set; // Используем set для автоматической уникальности и сортировки (если Point::operator< есть)

	for (const auto& [p1, p2] : data.refined_edges)
	{
		adj_list_tilde_V[p1].push_back(p2);
		adj_list_tilde_V[p2].push_back(p1);
		all_vertices_tilde_V_set.insert(p1);
		all_vertices_tilde_V_set.insert(p2);
	}

	// Шаг 2: Отсортировать списки смежности по полярному углу (против часовой стрелки)
	for (auto& [u, v] : adj_list_tilde_V)
	{
		Point center_node = u;
		std::ranges::sort(v,
			[&center_node](const Point& pa, const Point& pb) {
				return std::atan2(pa.y - center_node.y, pa.x - center_node.x) < std::atan2(pb.y - center_node.y, pb.x - center_node.x);
			});
	}

	// Шаг 3: Обход граней для нахождения всех ячеек (полигонов)
	std::vector<Polygon> found_cells_T;
	std::set<std::pair<Point, Point>> visited_half_edges; // Для отслеживания пройденных полуребер

	for (const Point& start_node_for_face : all_vertices_tilde_V_set)
	{
		if (!adj_list_tilde_V.contains(start_node_for_face))
			continue; // Узел может не иметь ребер (изолированный)

		for (const Point& first_neighbor : adj_list_tilde_V.at(start_node_for_face))
		{
			Point p_curr = start_node_for_face;
			Point p_next = first_neighbor;
			std::pair current_half_edge = { p_curr, p_next };

			if (visited_half_edges.contains(current_half_edge))
			{
				continue; // Это полуребро уже было начальной точкой обхода грани
			}

			Polygon current_cell_polygon;
			Point path_tracer_prev = p_curr; // Точка, из которой пришли в p_next
			Point path_tracer_curr = p_next; // Текущая вершина на границе грани

			// Трассировка одной грани
			while (true)
			{
				visited_half_edges.insert({ path_tracer_prev, path_tracer_curr });
				current_cell_polygon.push_back(path_tracer_prev); // Добавляем вершину, из которой исходит ребро грани

				const auto& neighbors_of_tracer_curr = adj_list_tilde_V.at(path_tracer_curr);
				if (neighbors_of_tracer_curr.empty())
					break; // Ошибка или конец графа

				// Ищем ребро (path_tracer_curr, path_tracer_prev) в списке соседей path_tracer_curr
				auto it_incoming_edge_neighbor = std::ranges::find(
					neighbors_of_tracer_curr, path_tracer_prev);
				if (it_incoming_edge_neighbor == neighbors_of_tracer_curr.end())
					break; // Ошибка

				size_t idx_incoming = std::distance(neighbors_of_tracer_curr.begin(), it_incoming_edge_neighbor);

				// Следующее ребро грани в обходе против часовой стрелки - это ребро перед входящим ребром (в отсортированном списке)
				Point next_node_for_face = neighbors_of_tracer_curr[(idx_incoming - 1 + neighbors_of_tracer_curr.size()) % neighbors_of_tracer_curr.size()];

				path_tracer_prev = path_tracer_curr;
				path_tracer_curr = next_node_for_face;

				if (path_tracer_prev == start_node_for_face && path_tracer_curr == first_neighbor)
				{
					break; // Замкнули цикл, вернулись к первому ребру грани
				}
				if (current_cell_polygon.size() > all_vertices_tilde_V_set.size() * 2)
				{
					// Защита от зацикливания
					LOG_WARN("Warning: Face traversal seems to be in an infinite loop.");
					current_cell_polygon.clear(); // Помечаем как невалидную
					break;
				}
			}

			if (!current_cell_polygon.empty() && current_cell_polygon.size() >= 3)
			{
				// Проверка на "внешнюю" грань. Внешняя грань обычно одна.
				// Простой эвристикой может быть проверка площади или ориентации.
				// Для корректной работы Contains() полигон должен быть с правильной ориентацией (например, против ЧС).
				// Алгоритм выше должен давать грани против ЧС, если atan2 отсортировал ребра против ЧС.
				// Пропустим сложную фильтрацию внешней грани пока.
				found_cells_T.push_back(current_cell_polygon);
			}
		}
	}

	data.all_tilde_V_cells = found_cells_T;

	// Шаг 4: Локализация точек s и t в найденных ячейках
	bool s_located = false;
	for (size_t i = 0; i < data.all_tilde_V_cells.size(); ++i)
	{
		const auto& cell_poly = data.all_tilde_V_cells[i];
		if (Contains(start_point_s, cell_poly))
		{
			data.s_cell_idx = i;
			s_located = true;
			break;
		}
	}

	bool t_located = false;
	for (size_t i = 0; i < data.all_tilde_V_cells.size(); ++i)
	{
		const auto& cell_poly = data.all_tilde_V_cells[i];
		if (Contains(target_point_t, cell_poly))
		{
			data.t_cell_idx = i;
			t_located = true;
			break;
		}
	}

	// Обработка случаев, если точки не найдены (могут быть на ребре или вне области)
	if (!s_located)
	{
		/* Опционально: логика для s на ребре или вне */
		static auto message = std::format("Failed to find any N_s points from START[{}, {}]", start_point_s.x, start_point_s.y);
		LOG_CRITICAL(message)
	}
	if (!t_located)
	{
		/* Опционально: логика для t на ребре или вне */
		static auto message = std::format("Failed to find any N_t points from GOAL[{}, {}]", target_point_t.x, target_point_t.y);
		LOG_CRITICAL(message)
	}

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
	const Point& target_point_t)
{
	m_lastGeneration = ConstructRefined(m_borderRect, polygons, start_point_s, target_point_t);
	return m_lastGeneration;
}

const VoronoiData& Voronoi::GetLastGeneration() const
{
	return m_lastGeneration;
}
