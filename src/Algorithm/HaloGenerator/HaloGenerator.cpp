//
// Created by User on 15.04.2025.
//

#include "HaloGenerator.h"

#include <format>
#include <iostream>
#include <stdexcept>

#include "../../Utility/Logger/Logger.h"
#include "../Math/Math.hpp"

using namespace tp::halos;
using namespace tp::shapes;
using namespace tp::math;

#ifndef M_PI
#define M_PI 3.14
#endif // !M_PI

namespace
{
/** @brief Вычисляет нормаль, повернутую на 90 градусов против часовой стрелки
 *  Если полигон по часовой стрелке, это внешняя нормаль
 */
Vec2D OutwardNormal(const Vec2D& segment_vec)
{
	Vec2D normal = { -segment_vec.y, segment_vec.x };

	return normal.Normalize();
}

/**
 * @brief Генерирует точки, аппроксимирующие дугу окружности.
 * @param center Центр дуги.
 * @param radius Радиус дуги.
 * @param startAngle Начальный угол в радианах.
 * @param endAngle Конечный угол в радианах.
 * @param numSegments Количество сегментов для аппроксимации.
 * @return Вектор точек дуги (включая начальную и конечную).
 */
std::vector<Point> GenerateArc(
	const Point& center,
	float radius,
	float startAngle,
	float endAngle,
	unsigned numSegments)
{
	std::vector<Point> arc_points;
	arc_points.reserve(numSegments + 1);

	// Нормализуем углы, чтобы end_angle был <= start_angle
	// Это упрощает вычисление шага и цикла
	while (endAngle > startAngle)
	{
		endAngle -= 2.0f * M_PI;
	}

	const float totalAngle = endAngle - startAngle;

	// Если угол слишком мал, просто добавляем две точки (или одну?)
	// Или если он почти 2*PI, обрабатываем как полный круг
	// Пока что используем стандартный расчет шага:
	const float angleStep = totalAngle / static_cast<float>(numSegments);

	for (int i = 0; i <= numSegments; ++i)
	{
		float currentAngle = startAngle + static_cast<float>(i) * angleStep;
		arc_points.push_back({ center.x + radius * std::cos(currentAngle),
			center.y + radius * std::sin(currentAngle) });
	}
	return arc_points;
}
} // namespace

HaloGenerator::HaloGenerator(float delta, unsigned segmentsPerArc)
	: m_delta(delta)
	, m_segmentsPerArc(segmentsPerArc)
{
	if (m_delta <= 0)
	{
		throw std::invalid_argument("Delta must be positive.");
	}
	if (m_segmentsPerArc < 3)
	{
		throw std::invalid_argument("Number of segments per arc must be at least 3 for proper approximation.");
	}
}

Polygon HaloGenerator::Generate(const Point& point) const
{
	// Используем достаточное количество сегментов для круга
	unsigned segmentsCount = std::max(4u, m_segmentsPerArc) * 4;
	Polygon halo;
	// Резервируем точное количество точек
	halo.reserve(segmentsCount);

	const auto angleStep = static_cast<float>((M_PI * 2) / segmentsCount);

	// Генерируем точки от 0 до (почти) 2*PI
	for (size_t i = 0; i < segmentsCount; ++i)
	{ // Строго < circle_segments
		float currentAngle = static_cast<float>(i) * angleStep;
		halo.push_back({ point.x + m_delta * std::cos(currentAngle),
			point.y + m_delta * std::sin(currentAngle) });
	}

	// Ожидаем, что этот метод теперь СТАБИЛЬНО генерирует CCW (отрицательная площадь)
	return halo;
}

Polygon HaloGenerator::Generate(const Segment& segment) const
{
	Vec2D vec = { segment.p1.x - segment.p2.x, segment.p1.y - segment.p2.y };

	// Если отрезок очень короткий, генерируем ореол как для точки (используем исправленную generate(Point))
	if (vec.LengthSquared() < SQUARED_EPSILON)
	{
		return Generate(segment.p1);
	}

	Vec2D direction = vec.Normalize();
	// Используем ту же функцию, что и в полигоне для единообразия
	// outwardNormal дает CCW нормаль ("влево" от направления)
	Vec2D normCW = OutwardNormal(direction); // Бывшая neg_norm
	Vec2D normCCW = { -normCW.x, -normCW.y }; // Бывшая norm

	// Углы этих нормалей
	float angleCCW = normCCW.Angle(); // Угол левой нормали
	float angleCW = normCW.Angle(); // Угол правой нормали

	// Генерируем дуги:
	// Дуга у p2: от левой (CCW) к правой (CW). generateArc идет CCW.
	std::vector<Point> p2Arc = GenerateArc(segment.p2, m_delta, angleCCW, angleCW, m_segmentsPerArc * 2);
	// Дуга у p1: от правой (CW) к левой (CCW). generateArc идет CCW.
	std::vector<Point> p1Arc = GenerateArc(segment.p1, m_delta, angleCW, angleCCW, m_segmentsPerArc * 2);

	// --- Остальная часть сборки сегмента остается прежней ---
	Polygon halo;
	halo.reserve(p1Arc.size() + p2Arc.size());
	halo.insert(halo.end(), p2Arc.begin(), p2Arc.end());
	if (p1Arc.size() > 1)
	{
		halo.insert(halo.end(), p1Arc.begin() + 1, p1Arc.end());
	}

	return halo; // Ожидаем CCW
}

Polygon HaloGenerator::Generate(const Polygon& polygon) const
{
	// --- ШАГ 1: Генерация всех дуг для каждой вершины ---
	size_t n = polygon.size();
	if (n == 1)
	{
		return Generate(polygon[0]);
	}
	if (n == 2)
	{
		return Generate(Segment{ polygon[0], polygon[1] });
	}

	std::vector<std::vector<Point>> allArcPoints(n);
	bool isGenerationSuccessful = true;
	for (size_t i = 0; i < n; ++i)
	{
		const Point& prev = polygon[(i + n - 1) % n];
		const Point& curr = polygon[i];
		const Point& next = polygon[(i + 1) % n];

		Vec2D inVector = curr - prev;
		Vec2D outVector = next - curr;

		// Обработка вырожденных ребер
		if (inVector.LengthSquared() < SQUARED_EPSILON || outVector.LengthSquared() < SQUARED_EPSILON)
		{
			LOG_WARN(std::format("Warning: Degenerate edge at vertex {}, skipping arc.", i));
			allArcPoints[i] = {};
			isGenerationSuccessful = false;
			continue;
		}

		// Генератор ожидает CCW вход для этих нормалей
		Vec2D inNormal = OutwardNormal(inVector);
		Vec2D outNormal = OutwardNormal(outVector);

		float startAngle = inNormal.Angle();
		float endAngle = outNormal.Angle();

		allArcPoints[i] = GenerateArc(
			curr, m_delta, startAngle, endAngle, m_segmentsPerArc);

		if (allArcPoints[i].empty())
		{
			LOG_WARN(std::format("GenerateArc returned empty points for vertex {}", i));
			isGenerationSuccessful = false;
		}
	}

	if (!isGenerationSuccessful)
	{
		LOG_ERROR("Failed to generate all arc segments for the halo");
		return {};
	}

	// --- ШАГ 2: Сборка итогового полигона ореола ---
	Polygon halo;
	halo.reserve(n * (m_segmentsPerArc + 2));

	for (size_t i = 0; i < n; ++i)
	{
		const std::vector<Point>& current_arc = allArcPoints[i];
		const std::vector<Point>& next_arc = allArcPoints[(i + 1) % n];

		if (halo.empty())
		{
			halo.insert(halo.end(), current_arc.begin(), current_arc.end()); // Все точки первой дуги
		}
		else
		{
			if (current_arc.size() > 1)
			{
				// Пропускаем ПЕРВУЮ точку текущей дуги
				halo.insert(halo.end(), current_arc.begin() + 1, current_arc.end());
			}
		}
		// Добавляем НАЧАЛЬНУЮ точку следующей дуги как соединительную
		if (!next_arc.empty())
		{
			halo.push_back(next_arc.front());
		}
	}

	// --- ШАГ 3: Очистка и проверка замыкания ---
	if (!halo.empty() && halo.size() > 1)
	{
		// Последняя добавленная точка была all_arc_points[0].front().
		// Самая первая точка была all_arc_points[0].front().
		// Проверяем, совпадают ли они.
		if ((halo.back() - halo.front()).LengthSquared() < SQUARED_EPSILON)
		{
			halo.pop_back(); // Удаляем дубликат замыкающей точки
		}
		else
		{
			LOG_WARN(std::format("Halo polygon not perfectly closed, forcibly closing polygon. "
								 "Last point: ({}, {}), First point: ({}, {})",
				halo.back().x, halo.back().y, halo.front().x, halo.front().y));
			halo.push_back(halo.front());
		}
	}

	// Ожидаем на выходе CCW полигон
	return halo;
}
