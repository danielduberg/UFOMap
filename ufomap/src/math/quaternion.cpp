#include <ufomap/math/quaternion.h>

namespace ufomap_math
{
Quaternion::Quaternion() : data_{ 1, 0, 0, 0 }
{
}

Quaternion::Quaternion(const Quaternion& other) : data_(other.data_)
{
}

Quaternion::Quaternion(double w, double x, double y, double z) : data_{ w, x, y, z }
{
}

Quaternion::Quaternion(const Vector3& other)
	: Quaternion(other.roll(), other.pitch(), other.yaw())
{
}

Quaternion::Quaternion(double roll, double pitch, double yaw)
{
	// double half_yaw = yaw * 0.5;
	// double half_pitch = pitch * 0.5;
	// double half_roll = roll * 0.5;
	// double cos_yaw = std::cos(half_yaw);
	// double sin_yaw = std::sin(half_yaw);
	// double cos_pitch = std::cos(half_pitch);
	// double sin_pitch = std::sin(half_pitch);
	// double cos_roll = std::cos(half_roll);
	// double sin_roll = std::sin(half_roll);
	// RPY
	// x() = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw;
	// y() = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw;
	// z() = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw;
	// w() = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw;
	// Euler
	// x() = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw;
	// y() = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw;
	// z() = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw;
	// w() = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw;

	double sroll = sin(roll);
	double spitch = sin(pitch);
	double syaw = sin(yaw);
	double croll = cos(roll);
	double cpitch = cos(pitch);
	double cyaw = cos(yaw);

	double m[3][3] = { // create rotational Matrix
										 { cyaw * cpitch, cyaw * spitch * sroll - syaw * croll,
											 cyaw * spitch * croll + syaw * sroll },
										 { syaw * cpitch, syaw * spitch * sroll + cyaw * croll,
											 syaw * spitch * croll - cyaw * sroll },
										 { -spitch, cpitch * sroll, cpitch * croll }
	};

	double _w = (double)(sqrt(std::max(0.0, 1 + m[0][0] + m[1][1] + m[2][2])) / 2.0);
	double _x = (double)(sqrt(std::max(0.0, 1 + m[0][0] - m[1][1] - m[2][2])) / 2.0);
	double _y = (double)(sqrt(std::max(0.0, 1 - m[0][0] + m[1][1] - m[2][2])) / 2.0);
	double _z = (double)(sqrt(std::max(0.0, 1 - m[0][0] - m[1][1] + m[2][2])) / 2.0);
	w() = _w;
	x() = (m[2][1] - m[1][2]) >= 0 ? fabs(_x) : -fabs(_x);
	y() = (m[0][2] - m[2][0]) >= 0 ? fabs(_y) : -fabs(_y);
	z() = (m[1][0] - m[0][1]) >= 0 ? fabs(_z) : -fabs(_z);
}

Quaternion::Quaternion(const Vector3& axis, double angle)
{
	double sa = sin(angle / 2);
	double ca = cos(angle / 2);
	x() = (double)(axis.x() * sa);
	y() = (double)(axis.y() * sa);
	z() = (double)(axis.z() * sa);
	w() = (double)ca;
}

Vector3 Quaternion::toEuler() const
{
	// create rotational matrix
	double n = norm();
	double s = n > 0 ? 2.0 / (n * n) : 0.0;

	double xs = x() * s;
	double ys = y() * s;
	double zs = z() * s;

	double wx = w() * xs;
	double wy = w() * ys;
	double wz = w() * zs;

	double xx = x() * xs;
	double xy = x() * ys;
	double xz = x() * zs;

	double yy = y() * ys;
	double yz = y() * zs;
	double zz = z() * zs;

	double m[3][3];

	m[0][0] = 1.0 - (yy + zz);
	m[1][1] = 1.0 - (xx + zz);
	m[2][2] = 1.0 - (xx + yy);

	m[1][0] = xy + wz;
	m[0][1] = xy - wz;

	m[2][0] = xz - wy;
	m[0][2] = xz + wy;
	m[2][1] = yz + wx;
	m[1][2] = yz - wx;

	double roll = (double)atan2(m[2][1], m[2][2]);
	double pitch = (double)atan2(-m[2][0], sqrt(m[2][1] * m[2][1] + m[2][2] * m[2][2]));
	double yaw = (double)atan2(m[1][0], m[0][0]);

	return Vector3(roll, pitch, yaw);
}

void Quaternion::toRotMatrix(std::vector<double>& rot_matrix_3_3) const
{
	// create rotational matrix
	double n = norm();
	double s = n > 0 ? 2.0 / (n * n) : 0.0;

	double xs = x() * s;
	double ys = y() * s;
	double zs = z() * s;

	double wx = w() * xs;
	double wy = w() * ys;
	double wz = w() * zs;

	double xx = x() * xs;
	double xy = x() * ys;
	double xz = x() * zs;

	double yy = y() * ys;
	double yz = y() * zs;
	double zz = z() * zs;

	double m[3][3];
	m[0][0] = 1.0 - (yy + zz);
	m[1][1] = 1.0 - (xx + zz);
	m[2][2] = 1.0 - (xx + yy);

	m[1][0] = xy + wz;
	m[0][1] = xy - wz;

	m[2][0] = xz - wy;
	m[0][2] = xz + wy;
	m[2][1] = yz + wx;
	m[1][2] = yz - wx;

	// double yy = 2.0 * y() * y();
	// double zz = 2.0 * z() * z();
	// double xx = 2.0 * x() * x();
	// double xy = 2.0 * x() * y();
	// double zw = 2.0 * z() * w();
	// double xz = 2.0 * x() * z();
	// double yw = 2.0 * y() * w();
	// double yz = 2.0 * y() * z();
	// double xw = 2.0 * x() * w();

	// double m[3][3];
	// // Row 1
	// m[0][0] = 1.0 - yy - zz;
	// m[0][1] = xy - zw;
	// m[0][2] = xz + yw;

	// // Row 2
	// m[1][0] = xy + zw;
	// m[1][1] = 1.0 - xx - zz;
	// m[1][2] = yz - xw;

	// // Row 3
	// m[2][0] = xz - yw;
	// m[2][1] = yz + xw;
	// m[2][2] = 1.0 - xx - yy;

	rot_matrix_3_3.clear();
	rot_matrix_3_3.resize(9, 0.0);
	for (unsigned int i = 0; i < 3; i++)
	{
		rot_matrix_3_3[i * 3] = m[i][0];
		rot_matrix_3_3[i * 3 + 1] = m[i][1];
		rot_matrix_3_3[i * 3 + 2] = m[i][2];
	}
}

const double& Quaternion::operator[](size_t index) const
{
	return data_[index];
}

double& Quaternion::operator[](size_t index)
{
	return data_[index];
}

double Quaternion::norm() const
{
	double n = 0;
	for (unsigned int i = 0; i < 4; i++)
	{
		n += operator[](i) * operator[](i);
	}
	return (double)sqrt(n);
}

Quaternion Quaternion::normalized() const
{
	Quaternion result(*this);
	result.normalize();
	return result;
}

Quaternion& Quaternion::normalize()
{
	double len = norm();
	if (len > 0)
		*this /= (double)len;
	return *this;
}

void Quaternion::operator/=(double x)
{
	for (unsigned int i = 0; i < 4; ++i)
	{
		operator[](i) /= x;
	}
}

Quaternion& Quaternion::operator=(const Quaternion& rhs)
{
	w() = rhs.w();
	x() = rhs.x();
	y() = rhs.y();
	z() = rhs.z();
	return *this;
}

bool Quaternion::operator==(const Quaternion& rhs) const
{
	for (unsigned int i = 0; i < 4; i++)
	{
		if (operator[](i) != rhs[i])
		{
			return false;
		}
	}
	return true;
}

bool Quaternion::operator!=(const Quaternion& rhs) const
{
	for (unsigned int i = 0; i < 4; i++)
	{
		if (operator[](i) != rhs[i])
		{
			return true;
		}
	}
	return false;
}

Quaternion Quaternion::operator*(const Quaternion& rhs) const
{
	return Quaternion(w() * rhs.w() - x() * rhs.x() - y() * rhs.y() - z() * rhs.z(),
										y() * rhs.z() - rhs.y() * z() + w() * rhs.x() + rhs.w() * x(),
										z() * rhs.x() - rhs.z() * x() + w() * rhs.y() + rhs.w() * y(),
										x() * rhs.y() - rhs.x() * y() + w() * rhs.z() + rhs.w() * z());
}

Quaternion Quaternion::operator*(const Vector3& v) const
{
	return *this * Quaternion(0, v(0), v(1), v(2));
}

Quaternion Quaternion::inversed() const
{
	return Quaternion(w(), -x(), -y(), -z());
}

Quaternion& Quaternion::inverse()
{
	x() = -x();
	y() = -y();
	z() = -z();
	return *this;
}

Vector3 Quaternion::rotate(const Vector3& v) const
{
	Quaternion q = *this * v * this->inversed();
	return Vector3(q.x(), q.y(), q.z());
}

const double& Quaternion::w() const
{
	return data_[0];
}
double& Quaternion::w()
{
	return data_[0];
}
const double& Quaternion::x() const
{
	return data_[1];
}
double& Quaternion::x()
{
	return data_[1];
}
const double& Quaternion::y() const
{
	return data_[2];
}
double& Quaternion::y()
{
	return data_[2];
}
const double& Quaternion::z() const
{
	return data_[3];
}
double& Quaternion::z()
{
	return data_[3];
}

}  // namespace ufomap_math