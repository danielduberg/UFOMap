#ifndef UFOMAP_MATH_POSE6_H
#define UFOMAP_MATH_POSE6_H

#include <ufomap/math/quaternion.h>
#include <ufomap/math/vector3.h>

namespace ufomap_math
{
class Pose6
{
public:
	Pose6();
	~Pose6();

	Pose6(const Pose6& other);
	Pose6(const Vector3& translation, const Quaternion& rotation);
	Pose6(double x, double y, double z, double roll, double pitch, double yaw);
	Pose6(double t_x, double t_y, double t_z, double r_w, double r_x, double r_y, double r_z);

	Pose6& operator=(const Pose6& other);

	bool operator==(const Pose6& other);
	bool operator!=(const Pose6& other);

	Vector3& translation();
	Vector3 translation() const;
	Quaternion& rotation();
	Quaternion rotation() const;

	double& x();
	double x() const;
	double& y();
	double y() const;
	double& z();
	double z() const;

	double roll() const;
	double pitch() const;
	double yaw() const;

	Vector3 transform(const Vector3& v) const;

	Pose6 inversed() const;
	Pose6& inverse();

	Pose6 operator*(const Pose6& other) const;

	Pose6& operator*=(const Pose6& other);

	double distance(const Pose6& other) const;

	double translationLength() const;

private:
	Vector3 translation_;
	Quaternion rotation_;
};
}  // namespace ufomap_math

#endif  // UFOMAP_MATH_POSE6_H