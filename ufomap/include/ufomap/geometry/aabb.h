#ifndef UFOMAP_GEOMETRY_AXIS_ALIGNED_BOUNDING_BOX_H
#define UFOMAP_GEOMETRY_AXIS_ALIGNED_BOUNDING_BOX_H

#include <ufomap/geometry/obb.h>
#include <ufomap/math/pose6.h>
#include <ufomap/math/vector3.h>

#include <exception>

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

	inline AABB(const AABB& aabb) : center(aabb.center), half_size(aabb.half_size)
	{
	}

	inline AABB(const Vector3& center, float half_size)
		: center(center), half_size(half_size, half_size, half_size)
	{
	}

	inline AABB(const Vector3& min, const Vector3& max) : half_size((max - min) / 2.0)
	{
		center = min + half_size;
	}

	inline Vector3 getMin() const
	{
		return center - half_size;
	}

	inline Vector3 getMax() const
	{
		return center + half_size;
	}

	inline void translate(const Vector3& translation)
	{
		throw std::logic_error("Function not yet implemented");
		// center += translation;
	}

	inline void rotate(const Vector3& rotation)
	{
		throw std::logic_error("Function not yet implemented");
		// TODO: Implement
	}

	inline OBB transform(const Pose6& transform) const
	{
		throw std::logic_error("Function not yet implemented");
		// TODO: Implement
	}
};
}  // namespace ufomap_geometry

#endif  // UFOMAP_GEOMETRY_AXIS_ALIGNED_BOUNDING_BOX_H