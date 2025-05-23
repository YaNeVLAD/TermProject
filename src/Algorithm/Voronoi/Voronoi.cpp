//
// Created by User on 19.04.2025.
//

#include "Voronoi.h"

#include <boost/polygon/voronoi.hpp>

#include "../../Utility/Logger/Logger.h"
#include "../Structures/BoostCompat.h"

#include <CGAL/Arr_segment_traits_2.h> // Добавить эту строку для отрезков
#include <CGAL/Arrangement_2.h> // Добавить эту строку
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/enum.h> // Для CGAL::ON_UNBOUNDED_SIDE и т.п.

#include <format>
#include <optional>

using namespace tp;
using namespace tp::shapes;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Arr_segment_traits_2<K> Traits_2;
typedef CGAL::Arrangement_2<Traits_2> Arrangement;
typedef K::Point_2 CGAL_Point;
typedef K::Segment_2 CGAL_Segment;

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
	// Можно добавить указатель на исходный полигон для доп. информации
	// const shapes::Polygon* source_polygon;

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
	Point getPsi(const shapes::Point& x) const
	{
		if (type == Type::VERTEX)
		{
			return vertex_site;
		}
		else
		{ // Type::SEGMENT
			const shapes::Vec2D ab = segment_site.p2 - segment_site.p1;
			const shapes::Vec2D ax = x - segment_site.p1; // Вектор от начала сегмента до точки x

			const float ab_len_sq = ab.LengthSquared();

			// Если сегмент вырожден в точку
			if (ab_len_sq < FLT_EPSILON * FLT_EPSILON)
			{
				return segment_site.p1;
			}

			// t = dot(ax, ab) / dot(ab, ab)
			float t = (ax.x * ab.x + ax.y * ab.y) / ab_len_sq;

			if (t < 0.0f)
			{
				return segment_site.p1; // Ближайшая точка - p1
			}
			else if (t > 1.0f)
			{
				return segment_site.p2; // Ближайшая точка - p2
			}
			// Проекция лежит на отрезке
			return segment_site.p1 + ab.Scale(t);
		}
	}
};

namespace
{
bool Contains(const Point& point, const Polygon& polygon)
{
	int intersections = 0;
	size_t n = polygon.size();

	for (size_t i = 0; i < n; ++i)
	{
		const auto& [ax, ay] = polygon[i];
		const auto& [bx, by] = polygon[(i + 1) % n];

		if (ay > point.y != (by > point.y) && point.x < (bx - ax) * (point.y - ay) / (by - ay) + ax)
		{
			intersections++;
		}
	}

	return intersections % 2 == 1;
}

struct BoostInputSourceInfo
{
	bool is_obstacle_related; // true, если это сегмент реального препятствия, false - если рамка
	Segment original_segment;

	explicit BoostInputSourceInfo(bool is_obs, Segment seg = {})
		: is_obstacle_related(is_obs)
		, original_segment(seg)
	{
	}
};

float distance_point_to_feature(const Point& p, const ObstacleFeature& feature)
{
	Point closest_on_feature = feature.getPsi(p);
	return (p - closest_on_feature).Length();
}

// ... (GetFeatureFromBoostSource можно упростить или встроить логику) ...

VoronoiData ConstructRefined(
	BorderRect borderRect,
	const std::vector<Polygon>& polygons,
	const std::vector<Polygon>& halos,
	const Point& start_point_s,
	const Point& target_point_t) // Halos используются для фильтрации финальных ребер
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

	Segment leftTop = { left, top };
	Segment rightTop = { right, top };
	Segment leftBottom = { left, bottom };
	Segment rightBottom = { right, bottom };

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
	std::set<std::pair<Point, Point>> added_edges_tracker;

	auto add_segment_if_valid =
		[&](Point p1, Point p2, const std::vector<Polygon>& current_polygons, const std::vector<Polygon>& current_halos) {
			if (p1 == p2)
				return;

			bool is_in_halo_or_poly = false;
			Point mid((p1.x + p2.x) * 0.5f, (p1.y + p2.y) * 0.5f);

			for (const auto& halo_poly : current_halos)
			{
				if (Contains(mid, halo_poly))
				{
					is_in_halo_or_poly = true;
					break;
				}
			}
			if (is_in_halo_or_poly)
				return;

			// Дополнительная проверка: не должно быть внутри исходных полигонов (если это не ребро самого полигона)
			// Эта проверка может быть сложной для ребер типа (i) и (ii), которые по определению соединяются с особенностями.
			// Пока оставим только проверку на гало.

			if (p2 < p1)
				std::swap(p1, p2); // Нормализация для set
			if (!added_edges_tracker.contains({ p1, p2 }))
			{
				data.refined_edges.emplace_back(Segment{ p1, p2 });
				added_edges_tracker.insert({ p1, p2 });
			}
		};

	// 1. Добавляем отфильтрованные стандартные ребра Вороного
	for (const auto& edge : vd.edges())
	{
		if (edge.is_infinite() || !edge.vertex0() || !edge.vertex1())
			continue;

		Point start_vd(edge.vertex0()->x(), edge.vertex0()->y());
		Point end_vd(edge.vertex1()->x(), edge.vertex1()->y());

		bool is_inside_obstacle = false;
		Point mid_vd((start_vd.x + end_vd.x) * 0.5f, (start_vd.y + end_vd.y) * 0.5f);
		for (const auto& poly : polygons)
		{ // Проверяем на попадание в исходные полигоны
			if (Contains(mid_vd, poly))
			{
				is_inside_obstacle = true;
				break;
			}
		}
		if (is_inside_obstacle)
			continue;

		add_segment_if_valid(start_vd, end_vd, polygons, halos);
	}

	// 2. Добавление ребер типа (i): x * \psi_o(x)
	// Для каждой вершины диаграммы Вороного...
	for (const auto& boost_v_ref : vd.vertices())
	{ // vd.vertices() возвращает коллекцию vertex_type объектов
		Point x_vor_vertex(boost_v_ref.x(), boost_v_ref.y());
		const voronoi_edge<double>* incident_edge = boost_v_ref.incident_edge();
		if (!incident_edge)
			continue;

		std::set<unsigned int> // Уникальные source_indices, уже обработанные для этой вершины VD
			processed_true_sites_for_this_vd_vertex; // Чтобы избежать дублирования из-за twin()

		const voronoi_edge<double>* current_iter_edge = incident_edge;
		do
		{
			const voronoi_cell<double>* cell = current_iter_edge->cell();
			if (cell)
			{
				unsigned int source_idx = cell->source_index();

				// Проверяем, не обработали ли мы уже этот сайт для данной вершины VD
				// (один сайт может быть доступен через несколько полуребер вокруг вершины)
				// Для большей надежности, можно было бы хранить пару (source_idx, category) или хэш от ObstacleFeature
				if (processed_true_sites_for_this_vd_vertex.count(source_idx * 10 + cell->source_category()))
				{ // Простой способ уникализации
					current_iter_edge = current_iter_edge->rot_next();
					continue;
				}

				if (source_info_map[source_idx].is_obstacle_related)
				{
					std::optional<ObstacleFeature> feature_o_for_psi = std::nullopt; // Особенность 'o', для которой считаем psi_o(x)

					// Категории источников из boost/polygon/detail/voronoi_structures.hpp (могут немного отличаться по названию)
					// typedef enum { GEOMETRY_CATEGORY_POINT, GEOMETRY_CATEGORY_SEGMENT } GEOMETRY_CATEGORY;
					// typedef enum { ..., SOURCE_CATEGORY_SINGLE_POINT, ..., SOURCE_CATEGORY_START_ENDPOINT, SOURCE_CATEGORY_END_ENDPOINT, SOURCE_CATEGORY_SEGMENT } SOURCE_CATEGORY;
					// Будем использовать числа, если точные enum'ы неясны, но лучше уточнить их для вашей версии Boost.
					// Допустим, 0 - точка, 1 - отрезок, 2 - начальная точка, 3 - конечная точка. (ЭТО ПРЕДПОЛОЖЕНИЕ!)

					int category = cell->source_category();
					const shapes::Segment& original_input_segment = source_info_map[source_idx].original_segment;

					// ПРОВЕРЬТЕ ЭТИ ЗНАЧЕНИЯ ENUM ИЛИ ИСПОЛЬЗУЙТЕ ИМЕНОВАННЫЕ КОНСТАНТЫ ИЗ BOOST HEADERS!
					const int cat_segment = SOURCE_CATEGORY_SEGMENT_START_POINT; // ПРЕДПОЛОЖЕНИЕ: boost::polygon::bits::SEGMENT_SOURCE_CATEGORY или аналогичное
					constexpr int cat_start_endpoint = SOURCE_CATEGORY_SEGMENT_START_POINT; // ПРЕДПОЛОЖЕНИЕ: ...bits::START_POINT_SOURCE_CATEGORY
					constexpr int cat_end_endpoint = SOURCE_CATEGORY_SEGMENT_END_POINT; // ПРЕДПОЛОЖЕНИЕ: ...bits::END_POINT_SOURCE_CATEGORY
					// const int cat_single_point = X; // Если вы видите эту категорию для конечных точек

					if (cell->incident_edge())
					{ // Если ячейка для отрезка
						feature_o_for_psi = ObstacleFeature(original_input_segment);
					}
					else if (category == cat_start_endpoint)
					{ // Если ячейка для начальной точки отрезка
						feature_o_for_psi = ObstacleFeature(original_input_segment.p1);
					}
					else if (category == cat_end_endpoint)
					{ // Если ячейка для конечной точки отрезка
						feature_o_for_psi = ObstacleFeature(original_input_segment.p2);
					}
					// Обработка SOURCE_CATEGORY_SINGLE_POINT (если она у вас встречается для конечных точек):
					// else if (category == cat_single_point) {
					//   // Здесь нужна логика для определения, какая из original_input_segment.p1 или .p2 является сайтом.
					//   // Это может потребовать анализа геометрии или более глубокого понимания API Boost.
					//   // Например, можно проверить, какая из точек (p1 или p2) ближе к x_vor_vertex,
					//   // так как x_vor_vertex должен быть равноудален от всех своих сайтов.
					//   // Это сложно и может быть неточным без доп. информации.
					//   // Пока пропустим, если только эти категории не покрывают ваш случай.
					// }

					if (feature_o_for_psi)
					{
						add_segment_if_valid(x_vor_vertex, feature_o_for_psi.value().getPsi(x_vor_vertex), polygons, halos);
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
		unsigned int source_idx = cell.source_index();

		// Пропускаем ячейки, не связанные с препятствиями
		if (source_idx >= source_info_map.size() || !source_info_map[source_idx].is_obstacle_related)
		{
			continue;
		}

		// Определяем особенность 'o' для текущей ячейки cell
		std::optional<ObstacleFeature> feature_o_cell = std::nullopt;
		const Segment& original_input_segment_for_cell = source_info_map[source_idx].original_segment;

		// Используйте ЗДЕСЬ РЕАЛЬНЫЕ ЗНАЧЕНИЯ ENUM из вашей версии Boost.Polygon!
		// Примерные значения (проверьте и исправьте):
		// namespace bpd = boost::polygon::detail; или boost::polygon::bits;
		// const int cat_segment = bpd::SEGMENT_SOURCE_CATEGORY; // или аналогичный
		constexpr int cat_start_endpoint = SOURCE_CATEGORY_SEGMENT_START_POINT; // или аналогичный
		constexpr int cat_end_endpoint = SOURCE_CATEGORY_SEGMENT_END_POINT; // или аналогичный

		// ЗАМЕНИТЕ ЭТИ ЧИСЛА НА РЕАЛЬНЫЕ КОНСТАНТЫ ENUM ИЗ BOOST!
		int category_cell = cell.source_category();
		// const int cat_segment_placeholder = 1;        // ЗАМЕНИТЬ!
		// const int cat_start_placeholder = 2;      // ЗАМЕНИТЬ!
		// const int cat_end_placeholder = 3;        // ЗАМЕНИТЬ!

		if (cell.incident_edge())
		{
			feature_o_cell = ObstacleFeature(original_input_segment_for_cell);
		}
		else if (category_cell == cat_start_endpoint)
		{
			feature_o_cell = ObstacleFeature(original_input_segment_for_cell.p1);
		}
		else if (category_cell == cat_end_endpoint)
		{
			feature_o_cell = ObstacleFeature(original_input_segment_for_cell.p2);
		}

		if (!feature_o_cell)
		{
			continue;
		}

		// Итерируем по первичным инцидентным ребрам этой ячейки 'V(o)'
		const voronoi_edge<double>* v_edge = cell.incident_edge();
		if (!v_edge)
			continue;

		const voronoi_edge<double>* start_iter_edge = v_edge;
		do
		{
			// Обрабатываем только первичные, конечные ребра.
			// is_secondary() может помочь отфильтровать внутренние ребра для отрезков-сайтов, если они есть.
			if (!v_edge->is_primary() || !v_edge->vertex0() || !v_edge->vertex1() /* || v_edge->is_secondary() */)
			{
				v_edge = v_edge->next();
				continue;
			}

			Point p_a(v_edge->vertex0()->x(), v_edge->vertex0()->y());
			Point p_b(v_edge->vertex1()->x(), v_edge->vertex1()->y());
			// Segment current_vd_edge_segment = {p_a, p_b}; // Не используется напрямую, но для понимания

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
			{ // Криволинейное ребро (парабола)
				// Для параболических ребер свойство выпуклости также сохраняется.
				// Минимум также будет на одном из концов хорды, которую представляет v_edge->vertex0/1(),
				// или в точке на параболе, где ее касательная параллельна "эффективной прямой" особенности 'o',
				// или в точке, где кривизна параболы "смотрит" на 'o'.
				// Это сложно. Самое простое приближение - использовать ту же логику, что и для линейных ребер,
				// т.е. проверить только конечные точки дискретизированного сегмента (хорды).
				// Более точный метод потребовал бы итерации по дискретизированным точкам параболы,
				// которые Boost использует для отрисовки, или аналитического решения.
				// Пока используем ту же эвристику конечных точек.
				float dist_pa_to_o = distance_point_to_feature(p_a, *feature_o_cell);
				float dist_pb_to_o = distance_point_to_feature(p_b, *feature_o_cell);

				if (dist_pa_to_o <= dist_pb_to_o)
				{
					x_star = p_a;
				}
				else
				{
					x_star = p_b;
				}
				// Примечание: для более точного результата на кривых ребрах, если Boost предоставляет
				// точки дискретизации для v_edge (например, через какой-либо метод вроде v_edge->discretize(...)),
				// следовало бы итерировать по этим точкам и найти среди них ту, что минимизирует расстояние до feature_o_cell.
			}

			Point psi_o_x_star = feature_o_cell->getPsi(x_star);
			add_segment_if_valid(x_star, psi_o_x_star, polygons, halos);

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
			std::pair<Point, Point> current_half_edge = { p_curr, p_next };

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
				{ // НУЖНО ДОБАВИТЬ ХОТЯ БЫ 1 БЛИЖАЙШУЮ ТОЧКУ
					break; // Замкнули цикл, вернулись к первому ребру грани
				}
				if (current_cell_polygon.size() > all_vertices_tilde_V_set.size() * 2)
				{ // Защита от зацикливания
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
			data.s_cell_idx = static_cast<int>(i);
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
			data.t_cell_idx = static_cast<int>(i);
			t_located = true;
			break;
		}
	}

	// Обработка случаев, если точки не найдены (могут быть на ребре или вне области)
	if (!s_located)
	{ /* Опционально: логика для s на ребре или вне */
		auto message = std::format("Failed to find any N_s points from START[{}, {}]", start_point_s.x, start_point_s.y);
		LOG_CRITICAL(message)
	}
	if (!t_located)
	{ /* Опционально: логика для t на ребре или вне */
		auto message = std::format("Failed to find any N_t points from GOAL[{}, {}]", target_point_t.x, target_point_t.y);
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
	const std::vector<Polygon>& halos,
	const Point& start_point_s,
	const Point& target_point_t)
{
	m_lastGeneration = ConstructRefined(m_borderRect, polygons, halos, start_point_s, target_point_t);
	return m_lastGeneration;
}

const VoronoiData& Voronoi::GetLastGeneration() const
{
	return m_lastGeneration;
}
