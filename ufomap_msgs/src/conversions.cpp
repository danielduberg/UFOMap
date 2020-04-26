#include <ufomap_msgs/conversions.h>

namespace ufomap_msgs
{
ufomap_math::Vector3 msgToUfomap(const ufomap_msgs::Point& point)
{
	return ufomap_math::Vector3(point.x, point.y, point.z);
}

ufomap_geometry::AABB msgToUfomap(const ufomap_msgs::AABB& aabb)
{
	return ufomap_geometry::AABB(msgToUfomap(aabb.center), msgToUfomap(aabb.half_size));
}

ufomap_geometry::Plane msgToUfomap(const ufomap_msgs::Plane& plane)
{
	return ufomap_geometry::Plane(msgToUfomap(plane.normal), plane.distance);
}

ufomap_geometry::Frustum msgToUfomap(const ufomap_msgs::Frustum& frustum)
{
	ufomap_geometry::Frustum f;
	for (size_t i = 0; i < frustum.planes.size(); ++i)
	{
		f.planes[i] = msgToUfomap(frustum.planes[i]);
	}
	return f;
}

ufomap_geometry::LineSegment msgToUfomap(const ufomap_msgs::LineSegment& line_segment)
{
	return ufomap_geometry::LineSegment(msgToUfomap(line_segment.start),
																			msgToUfomap(line_segment.end));
}

ufomap_geometry::OBB msgToUfomap(const ufomap_msgs::OBB& obb)
{
	return ufomap_geometry::OBB(msgToUfomap(obb.center), msgToUfomap(obb.half_size),
															msgToUfomap(obb.rotation));
}

ufomap_geometry::Ray msgToUfomap(const ufomap_msgs::Ray& ray)
{
	return ufomap_geometry::Ray(msgToUfomap(ray.origin), msgToUfomap(ray.direction));
}

ufomap_geometry::Sphere msgToUfomap(const ufomap_msgs::Sphere& sphere)
{
	return ufomap_geometry::Sphere(msgToUfomap(sphere.center), sphere.radius);
}

ufomap_geometry::BoundingVolume msgToUfomap(const ufomap_msgs::BoundingVolume& msg)
{
	ufomap_geometry::BoundingVolume bv;
	for (const ufomap_msgs::AABB& aabb : msg.aabbs)
	{
		bv.add(msgToUfomap(aabb));
	}
	for (const ufomap_msgs::Frustum& frustum : msg.frustums)
	{
		bv.add(msgToUfomap(frustum));
	}
	for (const ufomap_msgs::LineSegment& line_segment : msg.line_segments)
	{
		bv.add(msgToUfomap(line_segment));
	}
	for (const ufomap_msgs::OBB& obb : msg.obbs)
	{
		bv.add(msgToUfomap(obb));
	}
	for (const ufomap_msgs::Plane& plane : msg.planes)
	{
		bv.add(msgToUfomap(plane));
	}
	for (const ufomap_msgs::Point& point : msg.points)
	{
		bv.add(msgToUfomap(point));
	}
	for (const ufomap_msgs::Ray& ray : msg.rays)
	{
		bv.add(msgToUfomap(ray));
	}
	for (const ufomap_msgs::Sphere& sphere : msg.spheres)
	{
		bv.add(msgToUfomap(sphere));
	}
	return bv;
}

//
// UFOMap type to ROS message type
//

ufomap_msgs::Point ufomapToMsg(const ufomap_math::Vector3& point)
{
	ufomap_msgs::Point msg;
	msg.x = point.x();
	msg.y = point.y();
	msg.z = point.z();
	return msg;
}

ufomap_msgs::AABB ufomapToMsg(const ufomap_geometry::AABB& aabb)
{
	ufomap_msgs::AABB msg;
	msg.center = ufomapToMsg(aabb.center);
	msg.half_size = ufomapToMsg(aabb.half_size);
	return msg;
}

ufomap_msgs::Plane ufomapToMsg(const ufomap_geometry::Plane& plane)
{
	ufomap_msgs::Plane msg;
	msg.normal = ufomapToMsg(plane.normal);
	msg.distance = plane.distance;
	return msg;
}

ufomap_msgs::Frustum ufomapToMsg(const ufomap_geometry::Frustum& frustum)
{
	ufomap_msgs::Frustum msg;
	for (size_t i = 0; i < msg.planes.size(); ++i)
	{
		msg.planes[i] = ufomapToMsg(frustum.planes[i]);
	}
	return msg;
}

ufomap_msgs::LineSegment ufomapToMsg(const ufomap_geometry::LineSegment& line_segment)
{
	ufomap_msgs::LineSegment msg;
	msg.start = ufomapToMsg(line_segment.start);
	msg.end = ufomapToMsg(line_segment.end);
	return msg;
}

ufomap_msgs::OBB ufomapToMsg(const ufomap_geometry::OBB& obb)
{
	ufomap_msgs::OBB msg;
	msg.center = ufomapToMsg(obb.center);
	msg.half_size = ufomapToMsg(obb.half_size);
	msg.rotation = ufomapToMsg(obb.rotation);
	return msg;
}

ufomap_msgs::Ray ufomapToMsg(const ufomap_geometry::Ray& ray)
{
	ufomap_msgs::Ray msg;
	msg.origin = ufomapToMsg(ray.origin);
	msg.direction = ufomapToMsg(ray.direction);
	return msg;
}

ufomap_msgs::Sphere ufomapToMsg(const ufomap_geometry::Sphere& sphere)
{
	ufomap_msgs::Sphere msg;
	msg.center = ufomapToMsg(sphere.center);
	msg.radius = sphere.radius;
	return msg;
}

ufomap_msgs::BoundingVolume
ufomapToMsg(const ufomap_geometry::BoundingVolume& bounding_volume)
{
	ufomap_msgs::BoundingVolume msg;
	for (const ufomap_geometry::BoundingVar& bv : bounding_volume)
	{
		std::visit(
				[&msg](auto&& arg) -> void {
					using T = std::decay_t<decltype(arg)>;
					if constexpr (std::is_same_v<T, ufomap_geometry::AABB>)
					{
						msg.aabbs.push_back(ufomapToMsg(arg));
					}
					else if constexpr (std::is_same_v<T, ufomap_geometry::Frustum>)
					{
						msg.frustums.push_back(ufomapToMsg(arg));
					}
					else if constexpr (std::is_same_v<T, ufomap_geometry::LineSegment>)
					{
						msg.line_segments.push_back(ufomapToMsg(arg));
					}
					else if constexpr (std::is_same_v<T, ufomap_geometry::OBB>)
					{
						msg.obbs.push_back(ufomapToMsg(arg));
					}
					else if constexpr (std::is_same_v<T, ufomap_geometry::Plane>)
					{
						msg.planes.push_back(ufomapToMsg(arg));
					}
					else if constexpr (std::is_same_v<T, ufomap_math::Vector3>)
					{
						msg.points.push_back(ufomapToMsg(arg));
					}
					else if constexpr (std::is_same_v<T, ufomap_geometry::Ray>)
					{
						msg.rays.push_back(ufomapToMsg(arg));
					}
					else if constexpr (std::is_same_v<T, ufomap_geometry::Sphere>)
					{
						msg.spheres.push_back(ufomapToMsg(arg));
					}
				},
				bv);
	}
	return msg;
}
}  // namespace ufomap_msgs