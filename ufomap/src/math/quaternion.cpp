#include <ufomap/math/quaternion.h>

namespace ufomap_math
{
Quaternion::Quaternion() : data_{ 1, 0, 0, 0 }
{
}

Quaternion::Quaternion(const Quaternion& other) : data_(other.data_)
{
}

Quaternion::Quaternion(float w, float x, float y, float z) : data_{ w, x, y, z }
{
}

Quaternion::Quaternion(const Vector3& other)
	: Quaternion(other.roll(), other.pitch(), other.yaw())
{
}

Quaternion::Quaternion(float roll, float pitch, float yaw)
{
	float sroll = sin(roll);
	float spitch = sin(pitch);
	float syaw = sin(yaw);
	float croll = cos(roll);
	float cpitch = cos(pitch);
	float cyaw = cos(yaw);

	float m[3][3] = { // create rotational Matrix
										{ cyaw * cpitch, cyaw * spitch * sroll - syaw * croll,
											cyaw * spitch * croll + syaw * sroll },
										{ syaw * cpitch, syaw * spitch * sroll + cyaw * croll,
											syaw * spitch * croll - cyaw * sroll },
										{ -spitch, cpitch * sroll, cpitch * croll }
	};

	float _w = (float)(sqrt(std::max(0.0f, 1 + m[0][0] + m[1][1] + m[2][2])) / 2.0);
	float _x = (float)(sqrt(std::max(0.0f, 1 + m[0][0] - m[1][1] - m[2][2])) / 2.0);
	float _y = (float)(sqrt(std::max(0.0f, 1 - m[0][0] + m[1][1] - m[2][2])) / 2.0);
	float _z = (float)(sqrt(std::max(0.0f, 1 - m[0][0] - m[1][1] + m[2][2])) / 2.0);
	w() = _w;
	x() = (m[2][1] - m[1][2]) >= 0 ? fabs(_x) : -fabs(_x);
	y() = (m[0][2] - m[2][0]) >= 0 ? fabs(_y) : -fabs(_y);
	z() = (m[1][0] - m[0][1]) >= 0 ? fabs(_z) : -fabs(_z);
}

Quaternion::Quaternion(const Vector3& axis, float angle)
{
	float sa = sin(angle / 2);
	float ca = cos(angle / 2);
	x() = (float)(axis.x() * sa);
	y() = (float)(axis.y() * sa);
	z() = (float)(axis.z() * sa);
	w() = (float)ca;
}

Vector3 Quaternion::toEuler() const
{
	// create rotational matrix
	float n = norm();
	float s = n > 0 ? 2.0 / (n * n) : 0.0;

	float xs = x() * s;
	float ys = y() * s;
	float zs = z() * s;

	float wx = w() * xs;
	float wy = w() * ys;
	float wz = w() * zs;

	float xx = x() * xs;
	float xy = x() * ys;
	float xz = x() * zs;

	float yy = y() * ys;
	float yz = y() * zs;
	float zz = z() * zs;

	float m[3][3];

	m[0][0] = 1.0 - (yy + zz);
	m[1][1] = 1.0 - (xx + zz);
	m[2][2] = 1.0 - (xx + yy);

	m[1][0] = xy + wz;
	m[0][1] = xy - wz;

	m[2][0] = xz - wy;
	m[0][2] = xz + wy;
	m[2][1] = yz + wx;
	m[1][2] = yz - wx;

	float roll = (float)atan2(m[2][1], m[2][2]);
	float pitch = (float)atan2(-m[2][0], sqrt(m[2][1] * m[2][1] + m[2][2] * m[2][2]));
	float yaw = (float)atan2(m[1][0], m[0][0]);

	return Vector3(roll, pitch, yaw);
}

void Quaternion::toRotMatrix(std::vector<float>& rot_matrix_3_3) const
{
	// create rotational matrix
	float n = norm();
	float s = n > 0 ? 2.0 / (n * n) : 0.0;

	float xs = x() * s;
	float ys = y() * s;
	float zs = z() * s;

	float wx = w() * xs;
	float wy = w() * ys;
	float wz = w() * zs;

	float xx = x() * xs;
	float xy = x() * ys;
	float xz = x() * zs;

	float yy = y() * ys;
	float yz = y() * zs;
	float zz = z() * zs;

	float m[3][3];
	m[0][0] = 1.0 - (yy + zz);
	m[1][1] = 1.0 - (xx + zz);
	m[2][2] = 1.0 - (xx + yy);

	m[1][0] = xy + wz;
	m[0][1] = xy - wz;

	m[2][0] = xz - wy;
	m[0][2] = xz + wy;
	m[2][1] = yz + wx;
	m[1][2] = yz - wx;

	rot_matrix_3_3.clear();
	rot_matrix_3_3.resize(9, 0.);
	for (unsigned int i = 0; i < 3; i++)
	{
		rot_matrix_3_3[i * 3] = m[i][0];
		rot_matrix_3_3[i * 3 + 1] = m[i][1];
		rot_matrix_3_3[i * 3 + 2] = m[i][2];
	}
}

const float& Quaternion::operator[](size_t index) const
{
	return data_[index];
}

float& Quaternion::operator[](size_t index)
{
	return data_[index];
}

float Quaternion::norm() const
{
	float n = 0;
	for (unsigned int i = 0; i < 4; i++)
	{
		n += operator[](i) * operator[](i);
	}
	return (float)sqrt(n);
}

Quaternion Quaternion::normalized() const
{
	Quaternion result(*this);
	result.normalize();
	return result;
}

Quaternion& Quaternion::normalize()
{
	float len = norm();
	if (len > 0)
		*this /= (float)len;
	return *this;
}

void Quaternion::operator/=(float x)
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

const float& Quaternion::w() const
{
	return data_[0];
}
float& Quaternion::w()
{
	return data_[0];
}
const float& Quaternion::x() const
{
	return data_[1];
}
float& Quaternion::x()
{
	return data_[1];
}
const float& Quaternion::y() const
{
	return data_[2];
}
float& Quaternion::y()
{
	return data_[2];
}
const float& Quaternion::z() const
{
	return data_[3];
}
float& Quaternion::z()
{
	return data_[3];
}

}  // namespace ufomap