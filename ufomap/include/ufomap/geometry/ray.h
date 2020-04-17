#ifndef UFOMAP_GEOMETRY_RAY_H
#define UFOMAP_GEOMETRY_RAY_H

#include <ufomap/math/pose6.h>
#include <ufomap/math/vector3.h>

#include <exception>

using namespace ufomap_math;

namespace ufomap_geometry
{
struct Ray
{
	Vector3 origin;
	Vector3 direction;

	inline Ray() : direction(0.0, 0.0, 1.0)
	{
	}

	inline Ray(const Ray& ray) : origin(ray.origin), direction(ray.direction)
	{
	}

	inline Ray(const Vector3& origin, const Vector3& direction)
		: origin(origin), direction(direction.normalized())
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

#endif  // UFOMAP_GEOMETRY_RAY_H