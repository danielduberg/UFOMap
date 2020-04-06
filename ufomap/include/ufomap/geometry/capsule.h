#ifndef UFOMAP_GEOMETRY_CAPSULE_H
#define UFOMAP_GEOMETRY_CAPSULE_H

#include <ufomap/math/pose6.h>
#include <ufomap/math/vector3.h>

using namespace ufomap_math;

namespace ufomap_geometry
{
struct Capsule
{
	Vector3 start;
	Vector3 end;
	float radius;

	inline Capsule() : start(0.0, 0.0, 0.0), end(0.0, 0.0, 0.0), radius(0.0)
	{
	}

	inline Capsule(const Vector3& start, const Vector3& end, float radius)
		: start(start), end(end), radius(radius)
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

#endif  // UFOMAP_GEOMETRY_CAPSULE_H
