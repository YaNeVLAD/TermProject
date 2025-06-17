#ifndef HELPERS_H
#define HELPERS_H
#include <optional>
#include <vector>

#include "../../Utility/Logger/Logger.h"
#include "../../Utility/Shapes.hpp"
#include "../Math/Math.hpp"
#include "../Structures/BoostCompat.h"

#include <boost/geometry.hpp>
#include <format>

namespace tp
{
using namespace shapes;

template <typename Container, typename Func>
auto GroupBy(const Container& container, Func keySelector)
{
	// Выводим тип ключа из возвращаемого значения лямбда-функции
	using KeyType = decltype(keySelector(container.front()));

	// Создаем результирующую мапу
	std::map<KeyType, std::vector<typename Container::value_type>> resultMap;

	// Проходим по всем элементам входного контейнера
	for (const auto& item : container)
	{
		// 1. Получаем ключ, вызвав лямбду для текущего элемента
		// 2. Используем operator[] мапы:
		//    - Если ключ уже существует, получаем ссылку на существующий вектор.
		//    - Если ключа нет, он создается, и для него создается пустой вектор.
		// 3. Добавляем текущий элемент в соответствующий вектор.
		resultMap[keySelector(item)].push_back(item);
	}

	return resultMap;
}

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

Point ProjectPointToLine(const Point& p, const Point& l_p1, const Point& l_p2)
{
	Vec2D line_vec = l_p2 - l_p1;
	if (line_vec.LengthSquared() < FLT_EPSILON * FLT_EPSILON)
	{
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
	{
		return std::nullopt;
	}

	float t_num = (p1.x - p3.x) * (p3.y - p4.y) - (p1.y - p3.y) * (p3.x - p4.x);

	float t = t_num / den;

	return Point{ p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y) };
}

void DiscretizeRecursive(
	const Point& focus,
	const Point& directrix_l_p1, const Point& directrix_l_p2,
	const Point& arc_p0, const Point& arc_p2,
	size_t current_depth,
	std::vector<Point>& points_on_arc_accumulator)
{
	if (current_depth == 0 || arc_p0 == arc_p2)
	{
		if (points_on_arc_accumulator.empty() || points_on_arc_accumulator.back() != arc_p2)
		{
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

} // namespace tp

#endif // HELPERS_H
