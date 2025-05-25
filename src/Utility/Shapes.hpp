//
// Created by User on 15.04.2025.
//

#ifndef SHAPES_HPP
#define SHAPES_HPP

#include <cfloat>
#include <cmath>
#include <vector>

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

	auto operator<=>(const Segment&) const = default;
};

struct BorderRect
{
	float left, top, right, bottom;
};

using Polygon = std::vector<Point>;
} // namespace tp::shapes

#endif // SHAPES_HPP
