#include <ufomap/math/pose6.h>

namespace ufomap_math
{
Pose6::Pose6()
{
}

Pose6::~Pose6()
{
}

Pose6::Pose6(const Pose6& other)
	: translation_(other.translation_), rotation_(other.rotation_)
{
}

Pose6::Pose6(const Vector3& translation, const Quaternion& rotation)
	: translation_(translation), rotation_(rotation)
{
}

Pose6::Pose6(float x, float y, float z, float roll, float pitch, float yaw)
	: translation_(x, y, z), rotation_(roll, pitch, yaw)
{
}

Pose6& Pose6::operator=(const Pose6& rhs)
{
	translation_ = rhs.translation_;
	rotation_ = rhs.rotation_;
	return *this;
}

bool Pose6::operator==(const Pose6& other)
{
	return translation_ == other.translation_ && rotation_ == other.rotation_;
}
bool Pose6::operator!=(const Pose6& other)
{
	return translation_ != other.translation_ || rotation_ != other.rotation_;
}

Vector3& Pose6::translation()
{
	return translation_;
}
Vector3 Pose6::translation() const
{
	return translation_;
}
Quaternion& Pose6::rotation()
{
	return rotation_;
}
Quaternion Pose6::rotation() const
{
	return rotation_;
}

float& Pose6::x()
{
	return translation_[0];
}
float Pose6::x() const
{
	return translation_[0];
}
float& Pose6::y()
{
	return translation_[1];
}
float Pose6::y() const
{
	return translation_[1];
}
float& Pose6::z()
{
	return translation_[2];
}
float Pose6::z() const
{
	return translation_[2];
}

float Pose6::roll() const
{
	return rotation_.toEuler()[0];
}
float Pose6::pitch() const
{
	return rotation_.toEuler()[1];
}
float Pose6::yaw() const
{
	return rotation_.toEuler()[2];
}

Vector3 Pose6::transform(const Vector3& v) const
{
	Vector3 result = rotation_.rotate(v);
	result += translation_;
	return result;
}

Pose6 Pose6::inversed() const
{
	Pose6 result(*this);
	result.rotation_ = result.rotation_.inversed().normalized();
	result.translation_ = result.rotation_.rotate(-translation_);
	return result;
}

Pose6& Pose6::inverse()
{
	rotation_ = rotation_.inversed().normalized();
	translation_ = rotation_.rotate(-translation_);
	return *this;
}

Pose6 Pose6::operator*(const Pose6& other) const
{
	Quaternion rotation_new = rotation_ * other.rotation_;
	Vector3 transation_new = rotation_.rotate(other.translation_) + translation_;
	return Pose6(transation_new, rotation_new.normalized());
}

Pose6& Pose6::operator*=(const Pose6& other)
{
	translation_ += rotation_.rotate(other.translation_);
	rotation_ = rotation_ * other.rotation_;
	return *this;
}

float Pose6::distance(const Pose6& other) const
{
	double dist_x = x() - other.x();
	double dist_y = y() - other.y();
	double dist_z = z() - other.z();
	return sqrt((dist_x * dist_x) + (dist_y * dist_y) + (dist_z * dist_z));
}

float Pose6::translationLength() const
{
	return sqrt((x() * x()) + (y() * y()) + (z() * z()));
}

}  // namespace ufomap