#ifndef UFOMAP_GEOMETRY_COLLISION_CHECKS_H
#define UFOMAP_GEOMETRY_COLLISION_CHECKS_H

#include <ufomap/geometry/aabb.h>
#include <ufomap/geometry/bounding_volume.h>
#include <ufomap/geometry/frustum.h>
#include <ufomap/geometry/line_segment.h>
#include <ufomap/geometry/obb.h>
#include <ufomap/geometry/plane.h>
#include <ufomap/geometry/ray.h>
#include <ufomap/geometry/sphere.h>

namespace ufomap_geometry
{
// AABB
bool intersects(const AABB& aabb_1, const AABB& aabb_2);
bool intersects(const AABB& aabb, const Frustum& frustum);
bool intersects(const AABB& aabb, const LineSegment& line_segment);
bool intersects(const AABB& aabb, const OBB& obb);
bool intersects(const AABB& aabb, const Plane& plane);
bool intersects(const AABB& aabb, const Vector3& point);
bool intersects(const AABB& aabb, const Ray& ray);
bool intersects(const AABB& aabb, const Sphere& sphere);

// Frustum
bool intersects(const Frustum& frustum, const AABB& aabb);
bool intersects(const Frustum& frustum_1, const Frustum& frustum_2);  // TODO: Implement
bool intersects(const Frustum& frustum,
								const LineSegment& line_segment);  // TODO: Implement
bool intersects(const Frustum& frustum, const OBB& obb);
bool intersects(const Frustum& frustum, const Plane& plane);    // TODO: Implement
bool intersects(const Frustum& frustum, const Vector3& point);  // TODO: Implement
bool intersects(const Frustum& frustum, const Ray& ray);        // TODO: Implement
bool intersects(const Frustum& frustum, const Sphere& sphere);

// Line segment
bool intersects(const LineSegment& line_segment, const AABB& aabb);
bool intersects(const LineSegment& line_segment,
								const Frustum& frustum);  // TODO: Implement
bool intersects(const LineSegment& line_segment_1,
								const LineSegment& line_segment_2);  // TODO: Implement
bool intersects(const LineSegment& line_segment, const OBB& obb);
bool intersects(const LineSegment& line_segment, const Plane& plane);
bool intersects(const LineSegment& line_segment, const Vector3& point);
bool intersects(const LineSegment& line_segment, const Ray& ray);  // TODO: Implement
bool intersects(const LineSegment& line_segment, const Sphere& sphere);

// OBB
bool intersects(const OBB& obb, const AABB& aabb);
bool intersects(const OBB& obb, const Frustum& frustum);
bool intersects(const OBB& obb, const LineSegment& line_segment);
bool intersects(const OBB& obb_1, const OBB& obb_2);
bool intersects(const OBB& obb, const Plane& plane);
bool intersects(const OBB& obb, const Vector3& point);
bool intersects(const OBB& obb, const Ray& ray);
bool intersects(const OBB& obb, const Sphere& sphere);

// Plane
bool intersects(const Plane& plane, const AABB& aabb);
bool intersects(const Plane& plane, const Frustum& frustum);  // TODO: Implement
bool intersects(const Plane& plane, const LineSegment& line_segment);
bool intersects(const Plane& plane, const OBB& obb);
bool intersects(const Plane& plane_1, const Plane& plane_2);
bool intersects(const Plane& plane, const Vector3& point);
bool intersects(const Plane& plane, const Ray& ray);
bool intersects(const Plane& plane, const Sphere& sphere);

// Point
bool intersects(const Vector3& point, const AABB& aabb);
bool intersects(const Vector3& point, const Frustum& frustum);  // TODO: Implement
bool intersects(const Vector3& point, const LineSegment& line_segment);
bool intersects(const Vector3& point, const OBB& obb);
bool intersects(const Vector3& point, const Plane& plane);
bool intersects(const Vector3& point_1, const Vector3& point_2);  // TODO: Implement
bool intersects(const Vector3& point, const Ray& ray);
bool intersects(const Vector3& point, const Sphere& sphere);

// Ray
bool intersects(const Ray& ray, const AABB& aabb);
bool intersects(const Ray& ray, const Frustum& frustum);  // TODO: Implement
bool intersects(const Ray& ray,
								const LineSegment& line_segment);  // TODO: Implement
bool intersects(const Ray& ray, const OBB& obb);
bool intersects(const Ray& ray, const Plane& plane);
bool intersects(const Ray& ray, const Vector3& point);
bool intersects(const Ray& ray_1, const Ray& ray_2);  // TODO: Implement
bool intersects(const Ray& ray, const Sphere& sphere);

// Sphere
bool intersects(const Sphere& sphere, const AABB& aabb);
bool intersects(const Sphere& sphere, const Frustum& frustum);
bool intersects(const Sphere& sphere, const LineSegment& line_segment);
bool intersects(const Sphere& sphere, const OBB& obb);
bool intersects(const Sphere& sphere, const Plane& plane);
bool intersects(const Sphere& sphere, const Vector3& point);
bool intersects(const Sphere& sphere, const Ray& ray);
bool intersects(const Sphere& sphere_1, const Sphere& sphere_2);

// Inside
// bool inside(const AABB& aabb_1, const AABB& aabb_2);

// bool inside(const AABB& aabb, const Frustum& frustum);

// bool inside(const AABB& aabb, const LineSegment& line_segment);

// bool inside(const AABB& aabb, const OBB& obb);

// bool inside(const AABB& aabb, const Plane& plane);

// bool inside(const AABB& aabb, const Ray& ray);

// bool inside(const AABB& aabb, const Sphere& sphere);

}  // namespace ufomap_geometry

#endif  // UFOMAP_GEOMETRY_COLLISION_CHECKS_H