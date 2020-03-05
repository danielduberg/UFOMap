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

	Quaternion(float w, float x, float y, float z);

	Quaternion(const Vector3& other);

	Quaternion(float roll, float pitch, float yaw);

	Quaternion(const Vector3& axis, float angle);

	Vector3 toEuler() const;

	void toRotMatrix(std::vector<float>& rot_matrix_3_3) const;

	const float& operator[](size_t index) const;

	float& operator[](size_t index);

	float norm() const;

	Quaternion normalized() const;

	Quaternion& normalize();

	void operator/=(float x);

	Quaternion& operator=(const Quaternion& rhs);

	bool operator==(const Quaternion& other) const;

	bool operator!=(const Quaternion& other) const;

	Quaternion operator*(const Quaternion& rhs) const;

	Quaternion operator*(const Vector3& v) const;

	Quaternion inversed() const;

	Quaternion& inverse();

	Vector3 rotate(const Vector3& v) const;

	const float& w() const;
	float& w();
	const float& x() const;
	float& x();
	const float& y() const;
	float& y();
	const float& z() const;
	float& z();

private:
	std::array<float, 4> data_;
};
}  // namespace ufomap_math

#endif  // UFOMAP_MATH_QUATERNION_H