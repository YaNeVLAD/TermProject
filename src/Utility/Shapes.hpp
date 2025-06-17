//
// Created by User on 15.04.2025.
//

#ifndef SHAPES_HPP
#define SHAPES_HPP

#include <cfloat>
#include <cmath>
#include <vector>

#include "Color/Color.h"

namespace tp::shapes
{
struct Vec2D
{
	float x = 0, y = 0;

	[[nodiscard]] Vec2D Scale(float scale) const
	{
		return Vec2D{ x * scale, y * scale };
	}

	[[nodiscard]] float Length() const
	{
		return std::sqrt(x * x + y * y);
	}

	// Угол вектора в радианах [-PI, PI]
	[[nodiscard]] float Angle() const
	{
		if (std::abs(x) < DBL_EPSILON && std::abs(y) < DBL_EPSILON)
		{
			return 0.0f;
		}
		return std::atan2(y, x);
	}

	[[nodiscard]] float LengthSquared() const
	{
		return x * x + y * y;
	}

	[[nodiscard]] Vec2D Normalize() const
	{
		float len = Length();
		if (len < FLT_EPSILON)
		{
			return Vec2D{};
		}
		return Scale(1.f / len);
	}

	Vec2D operator+(const Vec2D& rhs) const
	{
		return Vec2D{ x + rhs.x, y + rhs.y };
	}

	Vec2D operator-(const Vec2D& rhs) const
	{
		return Vec2D{ x - rhs.x, y - rhs.y };
	}
};

struct Point
{
	float x = 0, y = 0;

	auto operator<=>(const Point&) const = default;

	bool operator<(const Point& other) const
	{
		if (x != other.x)
		{
			return x < other.x;
		}

		return y < other.y;
	}

	Vec2D operator-(const Point& other) const
	{
		return Vec2D{ x - other.x, y - other.y };
	}

	Vec2D operator+(const Point& other) const
	{
		return Vec2D{ x + other.x, y + other.y };
	}

	Point operator+(const Vec2D& vec) const
	{
		return Point{ x + vec.x, y + vec.y };
	}

	Point Midpoint(const Point& other) const
	{
		return Point{ (x + other.x) / 2.0f, (y + other.y) / 2.0f };
	}

	explicit operator Vec2D() const
	{
		return Vec2D{ x, y };
	}
};

struct Segment
{
	Point p1, p2;
	Color color;

	Segment() = default;
	Segment(const Segment&) = default;
	Segment(Segment&&) = default;

	Segment& operator=(const Segment&) = default;
	Segment& operator=(Segment&&) = default;

	Segment(const Point& p1, const Point& p2)
		: p1(p1)
		, p2(p2)
	{
	}

	Segment(const Point& p1, const Point& p2, Color color)
		: p1(p1)
		, p2(p2)
		, color(color)
	{
	}

	auto operator<=>(const Segment& other) const
	{
		if (auto cmp = p1 <=> other.p1; cmp != 0)
		{
			return cmp;
		}
		return p2 <=> other.p2;
	}
};

struct BorderRect
{
	float left, top, right, bottom;

	Point LeftTop() const
	{
		return { left, top };
	}

	Point RightTop() const
	{
		return { right, top };
	}

	Point LeftBottom() const
	{
		return { left, bottom };
	}

	Point RightBottom() const
	{
		return { right, bottom };
	}
};

using Polygon = std::vector<Point>;
} // namespace tp::shapes

#endif // SHAPES_HPP
