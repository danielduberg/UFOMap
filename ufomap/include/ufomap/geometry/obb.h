#ifndef UFOMAP_GEOMETRY_ORIENTED_BOUNDING_BOX_H
#define UFOMAP_GEOMETRY_ORIENTED_BOUNDING_BOX_H

#include <ufomap/math/pose6.h>
#include <ufomap/math/vector3.h>

#include <exception>

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

	inline OBB(const OBB& obb)
		: center(obb.center), half_size(obb.half_size), rotation(obb.rotation)
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

#endif  // UFOMAP_GEOMETRY_ORIENTED_BOUNDING_BOX_H