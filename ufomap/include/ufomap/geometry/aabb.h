#ifndef UFOMAP_GEOMETRY_AXIS_ALIGNED_BOUNDING_BOX_H
#define UFOMAP_GEOMETRY_AXIS_ALIGNED_BOUNDING_BOX_H

#include <ufomap/math/vector3.h>

using namespace ufomap_math;

namespace ufomap_geometry
{
struct AABB
{
	Vector3 center;
	Vector3 half_size;

	inline AABB()
	{
	}

	// inline AABB(const Vector3& center, const Vector3& half_size)
	// 	: center(center), half_size(half_size)
	// {
	// }

	inline AABB(const Vector3& min, const Vector3& max) : half_size((max - min) / 2.0)
	{
		center = min + half_size;
	}

	Vector3 getMin() const
	{
		return center - half_size;
	}

	Vector3 getMax() const
	{
		return center + half_size;
	}
};
}  // namespace ufomap_geometry

#endif  // UFOMAP_GEOMETRY_AXIS_ALIGNED_BOUNDING_BOX_H