#ifndef UFOMAP_TYPES_H
#define UFOMAP_TYPES_H

#include <ufomap/color.h>
#include <ufomap/math/vector3.h>

namespace ufomap
{
using Point3 = ufomap_math::Vector3;

class Point3RGB : public Point3
{
public:
	Point3RGB()
	{
	}

	Point3RGB(const Point3& point, const Color& color) : Point3(point), color_(color)
	{
	}

	Point3RGB(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b)
		: Point3(x, y, z), color_(r, g, b)
	{
	}

	Point3RGB(const Point3& point) : Point3(point), color_(255, 255, 255)
	{
	}

	Point3RGB(float x, float y, float z) : Point3(x, y, z), color_(255, 255, 255)
	{
	}

	Point3RGB(uint8_t r, uint8_t g, uint8_t b) : Point3(0, 0, 0), color_(r, g, b)
	{
	}

	Point3RGB(const Color& color) : Point3(0, 0, 0), color_(color)
	{
	}

	Point3RGB& operator=(const Point3RGB& rhs)
	{
		Point3::operator=(rhs);
		color_ = rhs.color_;
		return *this;
	}

	const Color& getColor() const
	{
		return color_;
	}

	Color& getColor()
	{
		return color_;
	}

	void setColor(const Color& new_color)
	{
		color_.r = new_color.r;
		color_.g = new_color.g;
		color_.b = new_color.b;
	}

	void setColor(uint8_t r, uint8_t g, uint8_t b)
	{
		color_.r = r;
		color_.g = g;
		color_.b = b;
	}

protected:
	Color color_;
};

class Point3I : public Point3
{
public:
	Point3I()
	{
	}

	Point3I(const Point3& point, float intensity) : Point3(point), intensity_(intensity)
	{
	}

	Point3I(float x, float y, float z, float intensity)
		: Point3(x, y, z), intensity_(intensity)
	{
	}

	Point3I(const Point3& point) : Point3(point), intensity_(0.0)
	{
	}

	Point3I(float x, float y, float z) : Point3(x, y, z), intensity_(0.0)
	{
	}

	Point3I(float intensity) : Point3(0, 0, 0), intensity_(intensity)
	{
	}

	Point3I& operator=(const Point3I& rhs)
	{
		Point3::operator=(rhs);
		intensity_ = rhs.intensity_;
		return *this;
	}

	float getIntensity() const
	{
		return intensity_;
	}

	float& getIntensity()
	{
		return intensity_;
	}

	void setIntesity(float new_intensity)
	{
		intensity_ = new_intensity;
	}

protected:
	float intensity_;
};
}  // namespace ufomap

#endif  // UFOMAP_TYPES_H