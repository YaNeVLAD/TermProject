//
// Created by User on 15.04.2025.
//

#ifndef COLOR_H
#define COLOR_H

#include <compare>
#include <cstdint>

namespace tp
{
struct Color
{
	Color();
	Color(Color&&) = default;
	Color(const Color&) noexcept = default;

	Color& operator=(const Color&) = default;
	Color& operator=(Color&&) = default;

	explicit Color(uint32_t hex);

	Color(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255);

	auto operator<=>(const Color&) const = default;

	static const Color Red;
	static const Color Green;
	static const Color Blue;
	static const Color Cyan;
	static const Color Magenta;
	static const Color White;
	static const Color Black;
	static const Color Yellow;
	static const Color Transparent;

	[[nodiscard]] uint32_t ToInt() const;

	explicit operator uint32_t() const
	{
		return ToInt();
	}

	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t a;
};
} // namespace tp

#endif // COLOR_H
