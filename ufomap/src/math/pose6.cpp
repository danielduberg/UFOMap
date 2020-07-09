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

Pose6::Pose6(double x, double y, double z, double roll, double pitch, double yaw)
	: translation_(x, y, z), rotation_(roll, pitch, yaw)
{
}

Pose6::Pose6(double t_x, double t_y, double t_z, double r_w, double r_x, double r_y, double r_z)
	: translation_(t_x, t_y, t_z), rotation_(r_w, r_x, r_y, r_z)
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

double& Pose6::x()
{
	return translation_[0];
}
double Pose6::x() const
{
	return translation_[0];
}
double& Pose6::y()
{
	return translation_[1];
}
double Pose6::y() const
{
	return translation_[1];
}
double& Pose6::z()
{
	return translation_[2];
}
double Pose6::z() const
{
	return translation_[2];
}

double Pose6::roll() const
{
	return rotation_.toEuler()[0];
}
double Pose6::pitch() const
{
	return rotation_.toEuler()[1];
}
double Pose6::yaw() const
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

double Pose6::distance(const Pose6& other) const
{
	double dist_x = x() - other.x();
	double dist_y = y() - other.y();
	double dist_z = z() - other.z();
	return sqrt((dist_x * dist_x) + (dist_y * dist_y) + (dist_z * dist_z));
}

double Pose6::translationLength() const
{
	return sqrt((x() * x()) + (y() * y()) + (z() * z()));
}

}  // namespace ufomap_math