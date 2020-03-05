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
};
}  // namespace ufomap_geometry

#endif  // UFOMAP_GEOMETRY_SPHERE_H