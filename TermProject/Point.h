#pragma once

#include <cmath>
#include <vector>

namespace path_finder
{

struct Point
{
	Point() = default;

	Point(double x, double y)
		: X(x)
		, Y(y)
	{
	}

	double X = 0, Y = 0;

	double Length() const
	{
		return std::hypot(X, Y);
	}

	Point Normalize()
	{
		double len = Length();

		if (len > 0)
		{
			return { X / len, Y / len };
		}
		return *this;
	}

	Point operator-(const Point& other) const
	{
		return { X - other.X, Y - other.Y };
	}

	Point operator+(const Point& other) const
	{
		return { X + other.X, Y + other.Y };
	}

	Point operator*(double scalar) const
	{
		return { X * scalar, Y * scalar };
	}

	bool operator!=(const Point& other) const
	{
		return X != other.X || Y != other.Y;
	}

	bool operator<(const Point& other) const
	{
		return (X < other.X) || (X == other.X && Y < other.Y);
	}
};

struct Segment
{
	Segment() = default;
	Segment(Point start, Point end);

	Point Start;
	Point End;
};

class Obstacle
{

public:
	Obstacle() = default;

	Obstacle(std::vector<Point> vertices)
		: m_Vertices(vertices)
	{
		m_Vertices.reserve(3);
	}

	bool Contains(const Point& point) const;
	Point ClosestPoint(const Point& point) const;
	bool Intersects(const Point& start, const Point& end) const;

	std::vector<Point>& Vertices();

private:
	std::vector<Point> m_Vertices;
};

} // namespace path_finder
