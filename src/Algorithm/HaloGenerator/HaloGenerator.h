//
// Created by User on 15.04.2025.
//

#ifndef HALO_GENERATOR_H
#define HALO_GENERATOR_H

#include <vector>

#include "../../Utility/Shapes.hpp"

namespace tp::halos
{
class HaloGenerator
{
public:
	/**
	 * @brief Конструктор генератора ореолов.
	 * @param delta Расстояние от границы фигуры до ореола (должно быть > 0).
	 * @param segmentsPerArc Количество линейных сегментов для аппроксимации дуги
	 * в 90 градусов (или полной дуги в случае точки/отрезка). Минимум 3.
	 */
	explicit HaloGenerator(float delta, unsigned segmentsPerArc = 16);

	/**
	 * @brief Генерирует ореол вокруг точки.
	 * @param point Входная точка.
	 * @return Полигон, представляющий окружность.
	 */
	[[nodiscard]] shapes::Polygon Generate(const shapes::Point& point) const;

	/**
	 * @brief Генерирует ореол вокруг отрезка.
	 * @param segment Входной отрезок.
	 * @return Полигон, представляющий капсулу.
	 */
	[[nodiscard]] shapes::Polygon Generate(const shapes::Segment& segment) const;

	/**
	 * @brief Генерирует ореол вокруг полигона.
	 * @param polygon Входной полигон. Порядок точек важен для нормалей.
	 * Предполагается, что полигон не самопересекающийся.
	 * @return Полигон, представляющий ореол. Может самопересекаться для вогнутых полигонов.
	 */
	[[nodiscard]] shapes::Polygon Generate(const shapes::Polygon& polygon) const;

private:
	float m_delta;
	unsigned m_segmentsPerArc;
};
} // namespace tp::halos

#endif // HALO_GENERATOR_H