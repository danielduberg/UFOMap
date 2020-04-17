#ifndef UFOMAP_GEOMETRY_ELLIPSOID_H
#define UFOMAP_GEOMETRY_ELLIPSOID_H

#include <ufomap/math/pose6.h>
#include <ufomap/math/vector3.h>

#include <exception>

using namespace ufomap_math;

namespace ufomap_geometry
{
struct Ellipsoid
{
	Vector3 center;
	Vector3 radius;

	inline Ellipsoid() : center(0.0, 0.0, 0.0), radius(0.0, 0.0, 0.0)
	{
	}

	inline Ellipsoid(const Vector3& center, const Vector3& radius)
		: center(center), radius(radius)
	{
	}

	inline void translate(const Vector3& translation)
	{
		throw std::logic_error("Function not yet implemented");
		// TODO: Implement
	}

	inline void rotate(const Vector3& rotation)
	{
		throw std::logic_error("Function not yet implemented");
		// TODO: Implement
	}

	inline void transform(const Pose6& transform)
	{
		throw std::logic_error("Function not yet implemented");
		// TODO: Implement
	}
};
}  // namespace ufomap_geometry

#endif  // UFOMAP_GEOMETRY_ELLIPSOID_H
