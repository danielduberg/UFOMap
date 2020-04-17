#ifndef UFOMAP_GEOMETRY_SPHERE_H
#define UFOMAP_GEOMETRY_SPHERE_H

#include <ufomap/math/pose6.h>
#include <ufomap/math/vector3.h>

#include <exception>

using namespace ufomap_math;

namespace ufomap_geometry
{
struct Sphere
{
	Vector3 center;
	float radius;

	inline Sphere() : center(0.0, 0.0, 0.0), radius(0.0)
	{
	}

	inline Sphere(const Sphere& sphere) : center(sphere.center), radius(sphere.radius)
	{
	}

	inline Sphere(const Vector3& center, float radius) : center(center), radius(radius)
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

#endif  // UFOMAP_GEOMETRY_SPHERE_H