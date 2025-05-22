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
	std::vector<shapes::Point> N_s; // Вершины ячейки T_s, содержащей начальную точку s
	std::vector<shapes::Point> N_t; // Вершины ячейки T_t, содержащей конечную точку t
	// Примечание: N_s и N_t могут быть пустыми, если точка s/t не найдена ни в одной ячейке
	// (например, если она за пределами borderRect или возникла ошибка при поиске ячеек).
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
