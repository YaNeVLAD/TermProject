#include "Details.hpp"

#include "Point.h"

double path_finder::details::Distance(Point first, Point second)
{
	double dx = second.X - first.X;
	double dy = second.Y - first.Y;

	return dx * dx + dy * dy;
}

bool path_finder::details::IsOnSegment(Point p, Point q, Point r)
{
	if (q.X <= std::max(p.X, r.X) && q.X >= std::min(p.X, r.X) && q.Y <= std::max(p.Y, r.Y) && q.Y >= std::min(p.Y, r.Y))
	{
		return true;
	}

	return false;
}

int path_finder::details::GetOrientation(Point p, Point q, Point r)
{
	int val = (q.Y - p.Y) * (r.X - q.X) - (q.X - p.X) * (r.Y - q.Y);

	if (val == 0)
	{
		return 0;
	}

	return (val > 0) ? 1 : 2;
}

bool path_finder::details::IsLineIntersects(Point p1, Point q1, Point p2, Point q2)
{
	int o1 = GetOrientation(p1, q1, p2);
	int o2 = GetOrientation(p1, q1, q2);
	int o3 = GetOrientation(p2, q2, p1);
	int o4 = GetOrientation(p2, q2, q1);

	if (o1 != o2 && o3 != o4)
	{
		return true;
	}

	if (o1 == 0 && IsOnSegment(p1, p2, q1))
	{
		return true;
	}

	if (o2 == 0 && IsOnSegment(p1, q2, q1))
	{
		return true;
	}

	if (o3 == 0 && IsOnSegment(p2, p1, q2))
	{
		return true;
	}

	if (o4 == 0 && IsOnSegment(p2, q1, q2))
	{
		return true;
	}

	return false;
}
