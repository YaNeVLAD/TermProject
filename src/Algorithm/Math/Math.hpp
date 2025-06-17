//
// Created by User on 15.04.2025.
//

#ifndef MATH_HPP
#define MATH_HPP

#include "../../Utility/Shapes.hpp"

namespace tp::math
{
constexpr double EPSILON = DBL_EPSILON;
constexpr double SQUARED_EPSILON = EPSILON * EPSILON;

inline float Distance(const shapes::Point& p1, const shapes::Point& p2)
{
	return (p2 - p1).Length();
}

inline float DistanceSquared(const shapes::Point& p1, const shapes::Point& p2)
{
	return (p2 - p1).LengthSquared();
}

inline int GetOrientation(const shapes::Vec2D& p, const shapes::Vec2D& q, const shapes::Vec2D& r)
{
	int val = static_cast<int>((q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y));
	if (val == 0)
	{
		return 0;
	}

	return val > 0 ? 1 : -1;
}

inline bool IsOnSegment(const shapes::Vec2D& p, const shapes::Vec2D& q, const shapes::Vec2D& r)
{
	bool hasSameX = q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x);
	bool hasSameY = q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y);

	if (hasSameX && hasSameY)
	{
		return true;
	}

	return false;
}

inline bool IsIntersect(const shapes::Vec2D& p1, const shapes::Vec2D& p2, const shapes::Vec2D& q1, const shapes::Vec2D& q2)
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

inline bool Contains(const shapes::Point& point, const shapes::Polygon& polygon)
{
	int intersections = 0;
	int n = polygon.size();

	if (n < 3)
	{
		return false;
	}

	for (size_t i = 0; i < n; ++i)
	{
		const auto& [ax, ay] = polygon[i];
		const auto& [bx, by] = polygon[(i + 1) % n];

		if ((ay <= point.y && point.y < by) || (by <= point.y && point.y < ay))
		{
			if (double intersect_x = ax + (point.y - ay) * (bx - ax) / (by - ay); intersect_x > point.x)
			{
				intersections++;
			}
		}
	}

	return intersections % 2 == 1;
}

/**
 * @brief Вычисляет квадрат минимального расстояния от точки p до отрезка ab.
 *
 * @param p Точка, от которой ищется расстояние.
 * @param a Первая точка отрезка.
 * @param b Вторая точка отрезка.
 * @return Квадрат расстояния от точки p до отрезка ab.
 */
inline float DistanceToSegmentSquared(const shapes::Point& p, const shapes::Point& a, const shapes::Point& b)
{
	const shapes::Vec2D ab = b - a;
	const shapes::Vec2D ap = p - a;

	const float ab_len_sq = ab.LengthSquared();

	// Если отрезок вырожден в точку (a и b совпадают или очень близки)
	if (ab_len_sq < FLT_EPSILON * FLT_EPSILON) // Сравниваем с квадратом эпсилон
	{
		return ap.LengthSquared(); // Расстояние от p до a (или b)
	}

	// Вычисляем проекцию вектора ap на вектор ab.
	// t = dot(ap, ab) / dot(ab, ab)
	// dot(ap, ab) = ap.x * ab.x + ap.y * ab.y
	const float dot_ap_ab = ap.x * ab.x + ap.y * ab.y;

	// Если проекция лежит "до" точки a (t < 0)
	if (dot_ap_ab <= 0.0f) // Используем <= для включения случая, когда p совпадает с a
	{
		return ap.LengthSquared(); // Ближайшая точка на отрезке - это a
	}

	// Если проекция лежит "после" точки b (t > 1, что эквивалентно dot(ap,ab) > dot(ab,ab))
	if (dot_ap_ab >= ab_len_sq) // Используем >= для включения случая, когда p совпадает с b
	{
		const shapes::Vec2D bp = p - b;
		return bp.LengthSquared(); // Ближайшая точка на отрезке - это b
	}

	// Проекция лежит на отрезке ab.
	// Расстояние от точки до прямой, на которой лежит отрезок.
	// Это высота треугольника pab с основанием ab.
	// Площадь треугольника = 0.5 * |ap.x * ab.y - ap.y * ab.x| (Z-компонента векторного произведения ap x ab)
	// Площадь треугольника = 0.5 * base * height = 0.5 * Length(ab) * height
	// height = |ap.x * ab.y - ap.y * ab.x| / Length(ab)
	// height_sq = (ap.x * ab.y - ap.y * ab.x)^2 / LengthSquared(ab)
	// Числитель: (ap.x * ab.y - ap.y * ab.x) - это также детерминант | ap.x  ap.y |
	//                                                              | ab.x  ab.y |
	const float cross_product_z = ap.x * ab.y - ap.y * ab.x;
	return (cross_product_z * cross_product_z) / ab_len_sq;
}
} // namespace tp::math

#endif // MATH_HPP
