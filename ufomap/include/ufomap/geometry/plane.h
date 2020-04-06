#ifndef UFOMAP_GEOMETRY_PLANE_H
#define UFOMAP_GEOMETRY_PLANE_H

#include <ufomap/math/vector3.h>

using namespace ufomap_math;

namespace ufomap_geometry
{
struct Plane
{
	Vector3 normal;
	float distance;

	inline Plane() : normal(1.0, 0.0, 0.0)
	{
	}

	inline Plane(const Plane& plane) : normal(plane.normal), distance(plane.distance)
	{
	}

	inline Plane(const Vector3& normal, float distance) : normal(normal), distance(distance)
	{
	}

	inline Plane(const Vector3& v_1, const Vector3& v_2, const Vector3& v_3)
	{
		Vector3 aux_1 = v_1 - v_2;
		Vector3 aux_2 = v_3 - v_2;
		normal = aux_2.cross(aux_1);
		normal.normalize();
		distance = -normal.dot(v_2);
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

#endif  // UFOMAP_GEOMETRY_PLANE_H