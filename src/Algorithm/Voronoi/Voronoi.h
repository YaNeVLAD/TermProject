//
// Created by User on 19.04.2025.
//

#ifndef VORONOI_H
#define VORONOI_H

#include "../../Utility/Shapes.hpp"

namespace tp
{
struct VoronoiData
{
	std::vector<shapes::Segment> refined_edges; // Все ребра $\tilde{\mathcal{V}}$
	std::vector<shapes::Polygon> all_tilde_V_cells; // ВСЕ внутренние ячейки $T$ диаграммы $\tilde{\mathcal{V}}$
													// Каждая Polygon - это список вершин ячейки.
	size_t s_cell_idx; // Индекс ячейки в all_tilde_V_cells, содержащей s (или -1)
	size_t t_cell_idx; // Индекс ячейки в all_tilde_V_cells, содержащей t (или -1)

	// Для удобства G1, можно добавить эти поля, которые заполняются на основе s_cell_idx/t_cell_idx
	// Либо PathFinder будет их извлекать сам из all_tilde_V_cells.
	// std::vector<shapes::Point> N_s_vertices;
	// std::vector<shapes::Point> N_t_vertices;

	VoronoiData()
		: s_cell_idx(-1)
		, t_cell_idx(-1)
	{
	} // Конструктор по умолчанию
};

class Voronoi
{
public:
	explicit Voronoi(shapes::BorderRect borderRect);

	[[nodiscard]] shapes::BorderRect GetBorderRect() const;

	void SetBorderRect(shapes::BorderRect borderRect);

	[[nodiscard]] VoronoiData Generate(
		const std::vector<shapes::Polygon>& polygons,
		const std::vector<shapes::Polygon>& halos,
		const shapes::Point& start_point_s,
		const shapes::Point& target_point_t);

	[[nodiscard]] const VoronoiData& GetLastGeneration() const;

private:
	shapes::BorderRect m_borderRect{};
	VoronoiData m_lastGeneration{};
};
} // namespace tp

#endif // VORONOI_H
