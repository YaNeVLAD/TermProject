#pragma once

namespace path_finder
{
struct Point;
} // namespace path_finder

namespace path_finder::details
{

double Distance(Point first, Point second);

bool IsOnSegment(Point p, Point q, Point r);

int GetOrientation(Point p, Point q, Point r);

bool IsLineIntersects(Point p1, Point q1, Point p2, Point q2);

} // namespace path_finder::details
