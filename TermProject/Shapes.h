#pragma once

#include <vector>

namespace tp::Shapes
{
struct Point
{
	float x = 0, y = 0;
};

struct Segment
{
	Point p1;
	Point p2;
};

using Polygon = std::vector<Point>;

} // namespace tp::Shapes
