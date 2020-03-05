#ifndef UFOMAP_COLOR_H
#define UFOMAP_COLOR_H

#include <cstdint>

namespace ufomap
{
/**
 * @brief RGB color
 *
 */
struct Color
{
	uint8_t r;
	uint8_t g;
	uint8_t b;

	Color() : r(255), g(255), b(255)
	{
	}

	Color(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b)
	{
	}

	Color(const Color& other) : r(other.r), g(other.g), b(other.b)
	{
	}

	Color& operator=(const Color& rhs)
	{
		r = rhs.r;
		g = rhs.g;
		b = rhs.b;
		return *this;
	}

	inline bool operator==(const Color& other) const
	{
		return other.r == r && other.g == g && other.b == b;
	}

	inline bool operator!=(const Color& other) const
	{
		return other.r != r || other.g != g || other.b != b;
	}
};
}  // namespace ufomap

#endif  // UFOMAP_COLOR_H
