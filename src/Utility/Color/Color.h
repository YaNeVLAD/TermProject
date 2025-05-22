//
// Created by User on 15.04.2025.
//

#ifndef COLOR_H
#define COLOR_H

#include <cstdint>

namespace tp
{
struct Color
{
	Color();
	Color(Color&& other) = default;
	Color(const Color& other) noexcept = default;

	explicit Color(uint32_t hex);

	Color(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255);

	Color& operator=(const Color& other) = default;
	Color& operator=(Color&& other) noexcept = default;

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