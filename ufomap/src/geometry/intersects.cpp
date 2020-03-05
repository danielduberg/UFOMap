#include <ufomap/geometry/intersects.h>

#include <limits>

namespace ufomap_geometry
{
// Help functions
bool intersectsLine(const AABB& aabb, const Ray& ray, float t_near, float t_far)
{
	Vector3 min = aabb.getMin();
	Vector3 max = aabb.getMax();

	for (int i = 0; i < 3; ++i)
	{
		if (0 != ray.direction[i])
		{
			float reciprocal_direction = 1.0 / ray.direction[i];
			float t1 = (min[i] - ray.origin[i]) * reciprocal_direction;
			float t2 = (max[i] - ray.origin[i]) * reciprocal_direction;

			if (t1 < t2)
			{
				t_near = std::max(t1, t_near);
				t_far = std::min(t2, t_far);
			}
			else
			{
				t_near = std::max(t2, t_near);
				t_far = std::min(t1, t_far);
			}

			if (t_near > t_far)
			{
				return false;
			}
		}
		else if (min[i] > ray.origin[i] || max[i] < ray.origin[i])
		{
			return false;
		}
	}
	return true;
}

Vector3 closestPoint(const Sphere& sphere, const Vector3& point)
{
	Vector3 sphere_to_point = point - sphere.center;
	sphere_to_point.normalize();
	sphere_to_point *= sphere.radius;
	return sphere_to_point + sphere.center;
}

Vector3 closestPoint(const AABB& aabb, const Vector3& point)
{
	Vector3 min = aabb.getMin();
	Vector3 max = aabb.getMax();
	return Vector3::clamp(point, min, max);
}

Vector3 closestPoint(const OBB& obb, const Vector3& point)
{
	Vector3 result = obb.center;
	Vector3 dir = point - obb.center;
	for (int i = 0; i < 3; ++i)
	{
		float distance = Vector3::dot(dir, obb.rotation);
		if (distance > obb.half_size[i])
		{
			distance = obb.half_size[i];
		}
		if (distance < -obb.half_size[i])  // TODO: Should this be else if?
		{
			distance = -obb.half_size[i];
		}
		result = result + (obb.rotation * distance);
	}
	return result;
}

Vector3 closestPoint(const Plane& plane, const Vector3& point)
{
	float distance = Vector3::dot(plane.normal, point) - plane.distance;
	return point - plane.normal * distance;
}

Vector3 closestPoint(const LineSegment& line_segement, const Vector3& point)
{
	Vector3 direction = line_segement.end - line_segement.start;
	float t = Vector3::dot(point - line_segement.start, direction) /
						Vector3::dot(direction, direction);
	t = fmaxf(t, 0.0f);
	t = fminf(t, 1.0f);
	return line_segement.start + direction * t;
}

Vector3 closestPoint(const Ray& ray, const Vector3& point)
{
	float t = Vector3::dot(point - ray.origin, ray.direction);
	t = fmaxf(t, 0.0f);
	return ray.origin + ray.direction * t;
}

// Classify
float classify(const AABB& aabb, const Plane& plane)
{
	float r = std::abs(aabb.half_size.x() * plane.normal.x()) +
						std::abs(aabb.half_size.y() * plane.normal.y()) +
						std::abs(aabb.half_size.z() * plane.normal.z());
	float d = Vector3::dot(plane.normal, aabb.center) + plane.distance;
	if (std::abs(d) < r)
	{
		return 0.0f;
	}
	else if (d < 0.0f)
	{
		return d + r;
	}
	return d - r;
}

float classify(const OBB& obb, const Plane& plane)
{
	Vector3 normal = plane.normal * obb.rotation;
	float r = std::abs(obb.half_size.x() * normal.x()) +
						std::abs(obb.half_size.y() * normal.y()) +
						std::abs(obb.half_size.z() * normal.z());
	float d = Vector3::dot(plane.normal, obb.center) + plane.distance;
	if (std::abs(d) < r)
	{
		return 0.0f;
	}
	else if (d < 0.0f)
	{
		return d + r;
	}
	return d - r;
}

std::pair<float, float> getInterval(const AABB& aabb, const Vector3& axis)
{
	Vector3 i = aabb.getMin();
	Vector3 a = aabb.getMax();

	Vector3 vertex[8] = { Vector3(i.x(), a.y(), a.z()), Vector3(i.x(), a.y(), i.z()),
												Vector3(i.x(), i.y(), a.z()), Vector3(i.x(), i.y(), i.z()),
												Vector3(a.x(), a.y(), a.z()), Vector3(a.x(), a.y(), i.z()),
												Vector3(a.x(), i.y(), a.z()), Vector3(a.x(), i.y(), i.z()) };

	std::pair<float, float> result;
	result.first = result.second = Vector3::dot(axis, vertex[0]);

	for (int i = 1; i < 8; ++i)
	{
		float projection = Vector3::dot(axis, vertex[i]);
		result.first = std::min(result.first, projection);
		result.second = std::max(result.first, projection);
	}

	return result;
}

std::pair<float, float> getInterval(const OBB& obb, const Vector3& axis)
{
	Vector3 vertex[8];

	Vector3 C = obb.center;     // OBB Center
	Vector3 E = obb.half_size;  // OBB Extents
	Vector3 A[] = {
		// OBB Axis
		Vector3(obb.rotation[0], 0, 0),
		Vector3(0, obb.rotation[1], 0),
		Vector3(0, 0, obb.rotation[2]),
	};

	vertex[0] = C + A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
	vertex[1] = C - A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
	vertex[2] = C + A[0] * E[0] - A[1] * E[1] + A[2] * E[2];
	vertex[3] = C + A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
	vertex[4] = C - A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
	vertex[5] = C + A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
	vertex[6] = C - A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
	vertex[7] = C - A[0] * E[0] - A[1] * E[1] + A[2] * E[2];

	std::pair<float, float> result;
	result.first = result.second = Vector3::dot(axis, vertex[0]);

	for (int i = 1; i < 8; ++i)
	{
		float projection = Vector3::dot(axis, vertex[i]);
		result.first = std::min(result.first, projection);
		result.second = std::max(result.first, projection);
	}

	return result;
}

bool overlapOnAxis(const AABB& aabb, const OBB& obb, const Vector3& axis)
{
	auto [a_min, a_max] = getInterval(aabb, axis);
	auto [b_min, b_max] = getInterval(obb, axis);
	return ((b_min <= a_max) && (a_min <= b_max));
}

bool overlapOnAxis(const OBB& obb_1, const OBB& obb_2, const Vector3& axis)
{
	auto [a_min, a_max] = getInterval(obb_1, axis);
	auto [b_min, b_max] = getInterval(obb_2, axis);
	return ((b_min <= a_max) && (a_min <= b_max));
}

// Intersects functions

bool intersects(const AABB& aabb_1, const AABB& aabb_2)
{
	Vector3 min_1 = aabb_1.getMin();
	Vector3 max_1 = aabb_1.getMax();
	Vector3 min_2 = aabb_2.getMin();
	Vector3 max_2 = aabb_2.getMax();
	return min_1.x() <= max_2.x() && min_1.y() <= max_2.y() && min_1.z() <= max_2.z() &&
				 min_2.x() <= max_1.x() && min_2.y() <= max_1.y() && min_2.z() <= max_1.z();
}

bool intersects(const AABB& aabb, const Frustum& frustum)
{
	// FIXME:
	for (int i = 0; i < 6; ++i)
	{
		float side = classify(aabb, frustum.planes[i]);
		if (side < 0)
		{
			return false;
		}
	}
	return true;
}

bool intersects(const Frustum& frustum, const AABB& aabb)
{
	return intersects(aabb, frustum);
}

bool intersects(const AABB& aabb, const LineSegment& line_segment)
{
	Ray ray;
	ray.origin = line_segment.start;
	ray.direction = (line_segment.end - line_segment.start);
	float length = ray.direction.norm();
	ray.direction /= length;
	return intersectsLine(aabb, ray, 0.0, length);
}

bool intersects(const LineSegment& line_segment, const AABB& aabb)
{
	return intersects(aabb, line_segment);
}

bool intersects(const AABB& aabb, const OBB& obb)
{
	Vector3 test[15] = { Vector3(1, 0, 0),  // AABB axis 1
											 Vector3(0, 1, 0),  // AABB axis 2
											 Vector3(0, 0, 1),  // AABB axis 3
											 Vector3(obb.rotation[0], 0, 0),
											 Vector3(0, obb.rotation[1], 0),
											 Vector3(0, 0, obb.rotation[2]) };

	for (int i = 0; i < 3; ++i)
	{  // Fill out rest of axis
		test[6 + i * 3 + 0] = Vector3::cross(test[i], test[0]);
		test[6 + i * 3 + 1] = Vector3::cross(test[i], test[1]);
		test[6 + i * 3 + 2] = Vector3::cross(test[i], test[2]);
	}

	for (int i = 0; i < 15; ++i)
	{
		if (!overlapOnAxis(aabb, obb, test[i]))
		{
			return false;  // Seperating axis found
		}
	}

	return true;  // Seperating axis not found
}

bool intersects(const OBB& obb, const AABB& aabb)
{
	return intersects(aabb, obb);
}

bool intersects(const AABB& aabb, const Plane& plane)
{
	float p_len = aabb.half_size.x() * std::abs(plane.normal.x()) +
								aabb.half_size.y() * std::abs(plane.normal.y()) +
								aabb.half_size.z() * std::abs(plane.normal.z());
	float distance = Vector3::dot(plane.normal, aabb.center) - plane.distance;
	return std::abs(distance) <= p_len;
}

bool intersects(const Plane& plane, const AABB& aabb)
{
	return intersects(aabb, plane);
}

bool intersects(const AABB& aabb, const Vector3& point)
{
	Vector3 min = aabb.getMin();
	Vector3 max = aabb.getMax();
	if (point.x() < min.x() || point.y() < min.y() || point.z() < min.z() ||
			point.x() > max.x() || point.y() > max.y() || point.z() > max.z())
	{
		return false;
	}
	return true;
}

bool intersects(const Vector3& point, const AABB& aabb)
{
	return intersects(aabb, point);
}

bool intersects(const AABB& aabb, const Ray& ray)
{
	return intersectsLine(aabb, ray, 0.0,
												std::numeric_limits<float>::infinity());  // TODO: infinity or
																																	// max?
}

bool intersects(const Ray& ray, const AABB& aabb)
{
	return intersects(aabb, ray);
}

bool intersects(const AABB& aabb, const Sphere& sphere)
{
	Vector3 closest_point = closestPoint(aabb, sphere.center);
	float distance_squared = (sphere.center - closest_point).squaredNorm();
	float radius_squared = sphere.radius * sphere.radius;
	return distance_squared < radius_squared;
}

bool intersects(const Sphere& sphere, const AABB& aabb)
{
	return intersects(aabb, sphere);
}

bool intersects(const OBB& obb, const Frustum& frustum)
{
	for (int i = 0; i < 6; ++i)
	{
		float side = classify(obb, frustum.planes[i]);
		if (side < 0)
		{
			return false;
		}
	}
	return true;
}

bool intersects(const Frustum& frustum, const OBB& obb)
{
	return intersects(obb, frustum);
}

bool intersects(const OBB& obb, const Vector3& point)
{
	Vector3 dir = point - obb.center;
	for (int i = 0; i < 3; ++i)
	{
		float distance = Vector3::dot(dir, obb.rotation);
		if (distance > obb.half_size[i])
		{
			return false;
		}
		if (distance < -obb.half_size[i])  // TODO: Should this be else if?
		{
			return false;
		}
	}
	return true;
}

bool intersects(const OBB& obb, const LineSegment& line_segment)
{
	Ray ray;
	ray.origin = line_segment.start;
	ray.direction = line_segment.end - line_segment.start;
	float line_length_squared = ray.direction.squaredNorm();
	if (line_length_squared < 0.0000001f)
	{
		return intersects(obb, line_segment.start);
	}
	ray.direction /= line_length_squared;  // Normalize

	// Begin ray casting

	Vector3 p = obb.center - ray.origin;

	Vector3 X(obb.rotation[0], 0, 0);
	Vector3 Y(0, obb.rotation[1], 0);
	Vector3 Z(0, 0, obb.rotation[2]);

	Vector3 f(Vector3::dot(X, ray.direction), Vector3::dot(Y, ray.direction),
						Vector3::dot(Z, ray.direction));

	Vector3 e(Vector3::dot(X, p), Vector3::dot(Y, p), Vector3::dot(Z, p));

	float t[6] = { 0, 0, 0, 0, 0, 0 };
	for (int i = 0; i < 3; ++i)
	{
		if (0.0 == f[i])  // TODO: Should be approximate equal
		{
			if (-e[i] - obb.half_size[i] > 0 || -e[i] + obb.half_size[i] < 0)
			{
				return false;
			}
			f[i] = 0.00001f;  // Avoid div by 0!
		}
		t[i * 2 + 0] = (e[i] + obb.half_size[i]) / f[i];  // tmin[x, y, z]
		t[i * 2 + 1] = (e[i] - obb.half_size[i]) / f[i];  // tmax[x, y, z]
	}

	float tmin = std::max(std::max(std::min(t[0], t[1]), std::min(t[2], t[3])),
												std::min(t[4], t[5]));
	float tmax = std::min(std::min(std::max(t[0], t[1]), std::max(t[2], t[3])),
												std::max(t[4], t[5]));

	// if tmax < 0, ray is intersecting AABB
	// but entire AABB is behing it's origin
	if (tmax < 0)
	{
		return false;
	}

	// if tmin > tmax, ray doesn't intersect AABB
	if (tmin > tmax)
	{
		return false;
	}

	// If tmin is < 0, tmax is closer
	float t_result = tmin;

	if (tmin < 0.0f)
	{
		t_result = tmax;
	}

	// End ray casting
	return t_result >= 0 && t_result * t_result <= line_length_squared;
}

bool intersects(const LineSegment& line_segment, const OBB& obb)
{
	return intersects(obb, line_segment);
}

bool intersects(const OBB& obb_1, const OBB& obb_2)
{
	Vector3 test[15] = {
		Vector3(obb_1.rotation[0], 0, 0), Vector3(0, obb_1.rotation[1], 0),
		Vector3(0, 0, obb_1.rotation[2]), Vector3(obb_2.rotation[0], 0, 0),
		Vector3(0, obb_2.rotation[1], 0), Vector3(0, 0, obb_2.rotation[2])
	};

	for (int i = 0; i < 3; ++i)
	{  // Fill out rest of axis
		test[6 + i * 3 + 0] = Vector3::cross(test[i], test[0]);
		test[6 + i * 3 + 1] = Vector3::cross(test[i], test[1]);
		test[6 + i * 3 + 2] = Vector3::cross(test[i], test[2]);
	}

	for (int i = 0; i < 15; ++i)
	{
		if (!overlapOnAxis(obb_1, obb_2, test[i]))
		{
			return false;  // Seperating axis found
		}
	}

	return true;  // Seperating axis not found
}

bool intersects(const OBB& obb, const Plane& plane)
{
	Vector3 rot[] = {
		Vector3(obb.rotation[0], 0, 0),
		Vector3(0, obb.rotation[1], 0),
		Vector3(0, 0, obb.rotation[2]),
	};
	Vector3 normal = plane.normal;

	// Project the half extents of the AABB onto the plane normal
	float p_len = obb.half_size.x() * std::fabs(Vector3::dot(normal, rot[0])) +
								obb.half_size.y() * std::fabs(Vector3::dot(normal, rot[1])) +
								obb.half_size.z() * std::fabs(Vector3::dot(normal, rot[2]));
	// Find the distance from the center of the OBB to the plane
	float distance = Vector3::dot(plane.normal, obb.center) - plane.distance;
	// Intersection occurs if the distance falls within the projected side
	return std::fabs(distance) <= p_len;
}

bool intersects(const Plane& plane, const OBB& obb)
{
	return intersects(obb, plane);
}

bool intersects(const OBB& obb, const Ray& ray)
{
	Vector3 p = obb.center - ray.origin;

	Vector3 X(obb.rotation[0], 0, 0);
	Vector3 Y(0, obb.rotation[1], 0);
	Vector3 Z(0, 0, obb.rotation[2]);

	Vector3 f(Vector3::dot(X, ray.direction), Vector3::dot(Y, ray.direction),
						Vector3::dot(Z, ray.direction));

	Vector3 e(Vector3::dot(X, p), Vector3::dot(Y, p), Vector3::dot(Z, p));

	float t[6] = { 0, 0, 0, 0, 0, 0 };
	for (int i = 0; i < 3; ++i)
	{
		if (0.0 == f[i])  // TODO: Should be approximate equal?
		{
			if (-e[i] - obb.half_size[i] > 0 || -e[i] + obb.half_size[i] < 0)
			{
				return false;
			}
			f[i] = 0.00001f;  // Avoid div by 0!
		}

		t[i * 2 + 0] = (e[i] + obb.half_size[i]) / f[i];  // tmin[x, y, z]
		t[i * 2 + 1] = (e[i] - obb.half_size[i]) / f[i];  // tmax[x, y, z]
	}

	float tmin = std::max(std::max(std::min(t[0], t[1]), std::min(t[2], t[3])),
												std::min(t[4], t[5]));
	float tmax = std::min(std::min(std::max(t[0], t[1]), std::max(t[2], t[3])),
												std::max(t[4], t[5]));

	// if tmax < 0, ray is intersecting AABB
	// but entire AABB is behing it's origin
	if (tmax < 0)
	{
		return false;
	}
	// if tmin > tmax, ray doesn't intersect AABB
	if (tmin > tmax)
	{
		return false;
	}
	return true;
}

bool intersects(const Ray& ray, const OBB& obb)
{
	return intersects(obb, ray);
}

bool intersects(const OBB& obb, const Sphere& sphere)
{
	Vector3 closest_point = closestPoint(obb, sphere.center);
	float distance_squared = (sphere.center - closest_point).squaredNorm();
	return distance_squared < sphere.radius * sphere.radius;
}

bool intersects(const Sphere& sphere, const OBB& obb)
{
	return intersects(obb, sphere);
}

bool intersects(const Vector3& point, const OBB& obb)
{
	return intersects(obb, point);
}

bool intersects(const Sphere& sphere, const Frustum& frustum)
{
	for (int i = 0; i < 6; ++i)
	{
		Vector3 normal = frustum.planes[i].normal;
		float distance = frustum.planes[i].distance;
		float side = Vector3::dot(sphere.center, normal) + distance;
		if (side < -sphere.radius)
		{
			return false;
		}
	}
	return true;
}

bool intersects(const Frustum& frustum, const Sphere& sphere)
{
	return intersects(sphere, frustum);
}

bool intersects(const Sphere& sphere, const LineSegment& line_segment)
{
	Vector3 closest_point = closestPoint(line_segment, sphere.center);
	float distance_squared = (sphere.center - closest_point).squaredNorm();
	return distance_squared <= (sphere.radius * sphere.radius);
}

bool intersects(const LineSegment& line_segment, const Sphere& sphere)
{
	return intersects(sphere, line_segment);
}

bool intersects(const Sphere& sphere, const Plane& plane)
{
	Vector3 closest_point = closestPoint(plane, sphere.center);
	float distance_squared = (sphere.center - closest_point).squaredNorm();
	return distance_squared < sphere.radius * sphere.radius;
}

bool intersects(const Plane& plane, const Sphere& sphere)
{
	return intersects(sphere, plane);
}

bool intersects(const Sphere& sphere, const Ray& ray)
{
	Vector3 e = sphere.center - ray.origin;
	float rSq = sphere.radius * sphere.radius;
	float eSq = e.squaredNorm();
	float a = Vector3::dot(e, ray.direction);
	return rSq - (eSq - a * a) >= 0.0;
}

bool intersects(const Ray& ray, const Sphere& sphere)
{
	return intersects(sphere, ray);
}

bool intersects(const Sphere& sphere_1, const Sphere& sphere_2)
{
	float radius_sum = sphere_1.radius + sphere_2.radius;
	float distance_squared = (sphere_1.center - sphere_2.center).squaredNorm();
	return distance_squared < radius_sum * radius_sum;
}

bool intersects(const Sphere& sphere, const Vector3& point)
{
	return (point - sphere.center).squaredNorm() < sphere.radius * sphere.radius;
}

bool intersects(const Vector3& point, const Sphere& sphere)
{
	return intersects(sphere, point);
}

bool intersects(const Plane& plane, const Vector3& point)
{
	return Vector3::dot(point, plane.normal) - plane.distance;
}

bool intersects(const Vector3& point, const Plane& plane)
{
	return intersects(plane, point);
}

bool intersects(const Plane& plane, const LineSegment& line_segment)
{
	Vector3 ab = line_segment.end - line_segment.start;
	float n_A = Vector3::dot(plane.normal, line_segment.start);
	float n_AB = Vector3::dot(plane.normal, ab);
	if (0.0 == n_AB)  // TODO: Almost equal?
	{
		return false;
	}
	float t = (plane.distance - n_A) / n_AB;
	return t >= 0.0 && t <= 1.0;
}

bool intersects(const LineSegment& line_segment, const Plane& plane)
{
	return intersects(plane, line_segment);
}

bool intersects(const Plane& plane_1, const Plane& plane_2)
{
	Vector3 d = Vector3::cross(plane_1.normal, plane_2.normal);
	return 0.0 != Vector3::dot(d, d);  // TODO: Almost not equal?
}

bool intersects(const Plane& plane, const Ray& ray)
{
	float nd = Vector3::dot(ray.direction, plane.normal);
	float pn = Vector3::dot(ray.origin, plane.normal);
	if (nd >= 0.0f)
	{
		return false;
	}
	float t = (plane.distance - pn) / nd;
	return t >= 0.0;
}

bool intersects(const Ray& ray, const Plane& plane)
{
	return intersects(plane, ray);
}

bool intersects(const LineSegment& line_segment, const Vector3& point)
{
	Vector3 closest_point = closestPoint(line_segment, point);
	float distance_squared = (closest_point - point).squaredNorm();
	return 0.0 == distance_squared;  // TODO: Almost equal?
}

bool intersects(const Vector3& point, const LineSegment& line_segment)
{
	return intersects(line_segment, point);
}

bool intersects(const Ray& ray, const Vector3& point)
{
	if (ray.origin == point)
	{
		return true;
	}
	Vector3 direction = point - ray.origin;
	direction.normalize();
	return 1.0 == Vector3::dot(direction, ray.direction);  // TODO: Almost equal?
}

bool intersects(const Vector3& point, const Ray& ray)
{
	return intersects(ray, point);
}

}  // namespace ufomap_geometry