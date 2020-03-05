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
};
}  // namespace ufomap_geometry

#endif  // UFOMAP_GEOMETRY_PLANE_H