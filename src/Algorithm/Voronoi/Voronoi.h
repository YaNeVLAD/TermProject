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
	std::vector<shapes::Segment> refined_edges;
	std::vector<shapes::Polygon> all_tilde_V_cells;

	size_t s_cell_idx;
	size_t t_cell_idx;

	VoronoiData()
		: s_cell_idx(-1)
		, t_cell_idx(-1)
	{
	}
};

class Voronoi
{
public:
	explicit Voronoi(shapes::BorderRect borderRect);

	[[nodiscard]] shapes::BorderRect GetBorderRect() const;

	void SetBorderRect(shapes::BorderRect borderRect);

	[[nodiscard]] VoronoiData Generate(
		const std::vector<shapes::Polygon>& polygons,
		const shapes::Point& start_point_s,
		const shapes::Point& target_point_t);

	[[nodiscard]] const VoronoiData& GetLastGeneration() const;

private:
	shapes::BorderRect m_borderRect{};
	VoronoiData m_lastGeneration{};
};
} // namespace tp

#endif // VORONOI_H
