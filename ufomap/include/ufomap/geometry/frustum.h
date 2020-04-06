#ifndef UFOMAP_GEOMETRY_FRUSTUM_H
#define UFOMAP_GEOMETRY_FRUSTUM_H

#include <ufomap/geometry/plane.h>
#include <ufomap/math/vector3.h>

#include <array>

using namespace ufomap_math;

namespace ufomap_geometry
{
struct Frustum
{
	std::array<Plane, 6> planes;

	inline Frustum()
	{
	}

	inline Frustum(const Frustum& frustum) : planes(frustum.planes)
	{
	}

	// TODO: Horizontal or vertical angle?
	inline Frustum(const Vector3& pos, const Vector3& target, const Vector3& up,
								 float vertical_angle, float horizontal_angle, float near_distance,
								 float far_distance)
	{
		float ratio = horizontal_angle / vertical_angle;

		// TODO: Check if correct
		float tang = std::tan(vertical_angle * 0.5);
		float near_height = near_distance * tang;
		float near_width = near_height * ratio;
		float far_height = far_distance * tang;
		float far_width = far_height * ratio;

		Vector3 Z = pos - target;
		Z.normalize();

		Vector3 X = Vector3::cross(up, Z);
		X.normalize();

		Vector3 Y = Vector3::cross(Z, X);

		Vector3 nc = pos - Z * near_distance;
		Vector3 fc = pos - Z * far_distance;

		Vector3 near_top_left = nc + Y * near_height - X * near_width;
		Vector3 near_top_right = nc + Y * near_height + X * near_width;
		Vector3 near_bottom_left = nc - Y * near_height - X * near_width;
		Vector3 near_bottom_right = nc - Y * near_height + X * near_width;

		Vector3 far_top_left = fc + Y * far_height - X * far_width;
		Vector3 far_top_right = fc + Y * far_height + X * far_width;
		Vector3 far_bottom_left = fc - Y * far_height - X * far_width;
		Vector3 far_bottom_right = fc - Y * far_height + X * far_width;

		top() = Plane(near_top_right, near_top_left, far_top_left);
		bottom() = Plane(near_bottom_left, near_bottom_right, far_bottom_right);
		left() = Plane(near_top_left, near_bottom_left, far_bottom_left);
		right() = Plane(near_bottom_right, near_top_right, far_bottom_right);
		near() = Plane(near_top_left, near_top_right, near_bottom_right);
		far() = Plane(far_top_right, far_top_left, far_bottom_left);
	}

	inline const Plane& top() const
	{
		return planes[0];
	}

	inline Plane& top()
	{
		return planes[0];
	}

	inline const Plane& bottom() const
	{
		return planes[1];
	}

	inline Plane& bottom()
	{
		return planes[1];
	}

	inline const Plane& left() const
	{
		return planes[2];
	}

	inline Plane& left()
	{
		return planes[2];
	}

	inline const Plane& right() const
	{
		return planes[3];
	}

	inline Plane& right()
	{
		return planes[3];
	}

	inline const Plane& near() const
	{
		return planes[4];
	}

	inline Plane& near()
	{
		return planes[4];
	}

	inline const Plane& far() const
	{
		return planes[5];
	}

	inline Plane& far()
	{
		return planes[5];
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

#endif  // UFOMAP_GEOMETRY_FRUSTUM_H