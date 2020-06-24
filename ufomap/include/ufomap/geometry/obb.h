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
	Quaternion rotation;

	inline OBB()
	{
	}

	inline OBB(const OBB& obb)
		: center(obb.center), half_size(obb.half_size), rotation(obb.rotation)
	{
	}

	inline OBB(const Vector3& center, const Vector3& half_size)
		: center(center), half_size(half_size)
	{
	}

	inline OBB(const Vector3& center, const Vector3& half_size, const Quaternion& rotation)
		: center(center), half_size(half_size), rotation(rotation)
	{
	}

	inline OBB(const Vector3& center, const Vector3& half_size, const Vector3& rotation)
		: center(center)
		, half_size(half_size)
		, rotation(rotation[0], rotation[1], rotation[2])
	{
	}
};
}  // namespace ufomap_geometry

#endif  // UFOMAP_GEOMETRY_ORIENTED_BOUNDING_BOX_H