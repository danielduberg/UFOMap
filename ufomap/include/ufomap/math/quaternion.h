#ifndef UFOMAP_MATH_QUATERNION_H
#define UFOMAP_MATH_QUATERNION_H

#include <ufomap/math/vector3.h>

#include <array>
#include <vector>

namespace ufomap_math
{
class Quaternion
{
public:
	Quaternion();

	Quaternion(const Quaternion& other);

	Quaternion(double w, double x, double y, double z);

	Quaternion(const Vector3& other);

	Quaternion(double roll, double pitch, double yaw);

	Quaternion(const Vector3& axis, double angle);

	Vector3 toEuler() const;

	void toRotMatrix(std::vector<double>& rot_matrix_3_3) const;

	const double& operator[](size_t index) const;

	double& operator[](size_t index);

	double norm() const;

	Quaternion normalized() const;

	Quaternion& normalize();

	void operator/=(double x);

	Quaternion& operator=(const Quaternion& rhs);

	bool operator==(const Quaternion& other) const;

	bool operator!=(const Quaternion& other) const;

	Quaternion operator*(const Quaternion& rhs) const;

	Quaternion operator*(const Vector3& v) const;

	Quaternion inversed() const;

	Quaternion& inverse();

	Vector3 rotate(const Vector3& v) const;

	const double& w() const;
	double& w();
	const double& x() const;
	double& x();
	const double& y() const;
	double& y();
	const double& z() const;
	double& z();

private:
	std::array<double, 4> data_;
};
}  // namespace ufomap_math

#endif  // UFOMAP_MATH_QUATERNION_H