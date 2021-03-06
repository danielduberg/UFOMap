#ifndef UFOMAP_GEOMETRY_RAY_H
#define UFOMAP_GEOMETRY_RAY_H

#include <ufomap/math/vector3.h>

using namespace ufomap_math;

namespace ufomap_geometry
{
struct Ray
{
	Vector3 origin;
	Vector3 direction;

	inline Ray() : direction(0.0, 0.0, 1.0)
	{
	}

	inline Ray(const Vector3& origin, const Vector3& direction)
		: origin(origin), direction(direction.normalized())
	{
	}
};
}  // namespace ufomap_geometry

#endif  // UFOMAP_GEOMETRY_RAY_H