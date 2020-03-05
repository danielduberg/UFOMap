#ifndef UFOMAP_GEOMETRY_FRUSTUM_H
#define UFOMAP_GEOMETRY_FRUSTUM_H

#include <ufomap/geometry/plane.h>
#include <ufomap/math/vector3.h>

using namespace ufomap_math;

namespace ufomap_geometry
{
struct Frustum
{
	Plane planes[6];

	inline Frustum()
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

	const Plane& top() const
	{
		return planes[0];
	}

	Plane& top()
	{
		return planes[0];
	}

	const Plane& bottom() const
	{
		return planes[1];
	}

	Plane& bottom()
	{
		return planes[1];
	}

	const Plane& left() const
	{
		return planes[2];
	}

	Plane& left()
	{
		return planes[2];
	}

	const Plane& right() const
	{
		return planes[3];
	}

	Plane& right()
	{
		return planes[3];
	}

	const Plane& near() const
	{
		return planes[4];
	}

	Plane& near()
	{
		return planes[4];
	}

	const Plane& far() const
	{
		return planes[5];
	}

	Plane& far()
	{
		return planes[5];
	}
};
}  // namespace ufomap_geometry

#endif  // UFOMAP_GEOMETRY_FRUSTUM_H