//
// Created by User on 15.04.2025.
//

#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <vector>

#include "../../Utility/Shapes.hpp"
#include "../Voronoi/Voronoi.h"

namespace tp
{
enum class GraphType
{
	G1,
	G2
};

class PathFinder
{
public:
	PathFinder(
		GraphType type,
		const VoronoiData& voronoi_data,
		const std::vector<shapes::Polygon>& obstacles,
		const shapes::Point& start, const shapes::Point& end);

	[[nodiscard]] std::vector<shapes::Segment> GetPath() const;

	[[nodiscard]] std::vector<shapes::Segment> GetEdges() const;

private:
	std::vector<shapes::Segment> m_edges;
	std::vector<shapes::Point> m_path;
};
} // namespace tp

#endif // PATHFINDER_H
