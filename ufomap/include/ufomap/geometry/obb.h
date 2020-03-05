#ifndef UFOMAP_GEOMETRY_ORIENTED_BOUNDING_BOX_H
#define UFOMAP_GEOMETRY_ORIENTED_BOUNDING_BOX_H

#include <ufomap/math/quaternion.h>
#include <ufomap/math/vector3.h>

using namespace ufomap_math;

namespace ufomap_geometry
{
struct OBB
{
	Vector3 center;
	Vector3 half_size;
	Vector3 rotation;

	inline OBB()
	{
	}

	inline OBB(const Vector3& center, const Vector3& half_size)
		: center(center), half_size(half_size)
	{
	}

	inline OBB(const Vector3& center, const Vector3& half_size, const Vector3& rotation)
		: center(center), half_size(half_size), rotation(rotation)
	{
	}
};
}  // namespace ufomap_geometry

#endif  // UFOMAP_GEOMETRY_ORIENTED_BOUNDING_BOX_H