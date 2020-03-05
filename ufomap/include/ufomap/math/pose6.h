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
	Pose6(float x, float y, float z, float roll, float pitch, float yaw);

	Pose6& operator=(const Pose6& other);

	bool operator==(const Pose6& other);
	bool operator!=(const Pose6& other);

	Vector3& translation();
	Vector3 translation() const;
	Quaternion& rotation();
	Quaternion rotation() const;

	float& x();
	float x() const;
	float& y();
	float y() const;
	float& z();
	float z() const;

	float roll() const;
	float pitch() const;
	float yaw() const;

	Vector3 transform(const Vector3& v) const;

	Pose6 inversed() const;
	Pose6& inverse();

	Pose6 operator*(const Pose6& other) const;

	Pose6& operator*=(const Pose6& other);

	float distance(const Pose6& other) const;

	float translationLength() const;

private:
	Vector3 translation_;
	Quaternion rotation_;
};
}  // namespace ufomap_math

#endif  // UFOMAP_MATH_POSE6_H