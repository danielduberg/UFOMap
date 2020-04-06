#ifndef UFOMAP_GEOMETRY_TRIANGLE_H
#define UFOMAP_GEOMETRY_TRIANGLE_H

#include <ufomap/math/pose6.h>
#include <ufomap/math/vector3.h>

using namespace ufomap_math;

namespace ufomap_geometry
{
struct Triangle
{
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

#endif  // UFOMAP_GEOMETRY_TRIANGLE_H