#include "Point.h"
#include "Details.hpp"
#include <cmath>
#include <limits>

namespace path_finder
{

bool Obstacle::Contains(const Point& point) const
{
	int intersections = 0;
	size_t n = m_Vertices.size();

	for (size_t i = 0; i < n; ++i)
	{
		const Point& p1 = m_Vertices[i];
		const Point& p2 = m_Vertices[(i + 1) % n];

		if ((p1.Y > point.Y) != (p2.Y > point.Y) && point.X < (p2.X - p1.X) * (point.Y - p1.Y) / (p2.Y - p1.Y) + p1.X)
		{
			intersections++;
		}
	}
	return intersections % 2 == 1;
}

Point Obstacle::ClosestPoint(const Point& point) const
{
	Point closest;
	double minDistance = std::numeric_limits<double>::max();

	size_t n = m_Vertices.size();
	for (size_t v = 0; v < n; ++v)
	{
		const Point& first = m_Vertices[v];
		const Point& second = m_Vertices[(v + 1) % n];

		double dx = second.X - first.X;
		double dy = second.Y - first.Y;
		double length = details::Distance(first, second);

		double t = ((point.X - first.X) * dx + (point.Y - first.Y) * dy) / length;
		t = std::max(0.0, std::min(1.0, t));

		Point proj = { first.X + t * dx, first.Y + t * dy };
		double dist = std::hypot(point.X - proj.X, point.Y - proj.Y);

		if (dist < minDistance)
		{
			minDistance = dist;
			closest = proj;
		}
	}

	return closest;
}

bool Obstacle::Intersects(const Point& start, const Point& end) const
{
	for (size_t i = 0; i < m_Vertices.size(); ++i)
	{
		Point v1 = m_Vertices[i];
		Point v2 = m_Vertices[(i + 1) % m_Vertices.size()];

		if (details::IsLineIntersects(start, end, v1, v2))
		{
			return true;
		}
	}
	return false;
}

std::vector<Point>& Obstacle::Vertices()
{
	return m_Vertices;
}

Segment::Segment(Point start, Point end)
	: Start(start)
	, End(end)
{
}

} // namespace path_finder
