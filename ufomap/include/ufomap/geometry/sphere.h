#ifndef UFOMAP_GEOMETRY_SPHERE_H
#define UFOMAP_GEOMETRY_SPHERE_H

#include <ufomap/math/vector3.h>

using namespace ufomap_math;

namespace ufomap_geometry
{
struct Sphere
{
	Vector3 center;
	float radius;

	inline Sphere() : center(0.0, 0.0, 0.0), radius(0.0)
	{
	}

	inline Sphere(const Vector3& center, float radius) : center(center), radius(radius)
	{
	}

	inline void translate(const Vector3& translation)
	{
		// TODO: Implement
	}

	inline void rotate(const Vector3& rotation)
	{
		// TODO: Implement
	}

	inline void transform(const Pose6& transform)
	{
		// TODO: Implement
	}
};
}  // namespace ufomap_geometry

#endif  // UFOMAP_GEOMETRY_SPHERE_H