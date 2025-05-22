//
// Created by User on 15.04.2025.
//

#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <vector>

#include "../../Utility/Shapes.hpp"

namespace tp
{
class PathFinder
{
public:
	PathFinder(
		const std::vector<shapes::Segment>& segments,
		const std::vector<shapes::Polygon>& obstacles,
		const shapes::Point& start, const shapes::Point& end,
		const std::vector<shapes::Point>& N_s, const std::vector<shapes::Point>& N_t);

	[[nodiscard]] std::vector<shapes::Segment> GetPath() const;

private:
	std::vector<shapes::Point> m_path;
};
} // namespace tp

#endif // PATHFINDER_H
