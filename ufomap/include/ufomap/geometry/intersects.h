#ifndef UFOMAP_GEOMETRY_INTERSECTS_H
#define UFOMAP_GEOMETRY_INTERSECTS_H

#include <ufomap/geometry/aabb.h>
#include <ufomap/geometry/frustum.h>
#include <ufomap/geometry/line_segment.h>
#include <ufomap/geometry/obb.h>
#include <ufomap/geometry/plane.h>
#include <ufomap/geometry/ray.h>
#include <ufomap/geometry/sphere.h>

namespace ufomap_geometry
{
bool intersects(const AABB& aabb_1, const AABB& aabb_2);

bool intersects(const AABB& aabb, const Frustum& frustum);

bool intersects(const Frustum& frustum, const AABB& aabb);

bool intersects(const AABB& aabb, const LineSegment& line_segment);

bool intersects(const LineSegment& line_segment, const AABB& aabb);

bool intersects(const AABB& aabb, const OBB& obb);

bool intersects(const OBB& obb, const AABB& aabb);

bool intersects(const AABB& aabb, const Plane& plane);

bool intersects(const Plane& plane, const AABB& aabb);

bool intersects(const AABB& aabb, const Vector3& point);

bool intersects(const Vector3& point, const AABB& aabb);

bool intersects(const AABB& aabb, const Ray& ray);

bool intersects(const Ray& ray, const AABB& aabb);

bool intersects(const AABB& aabb, const Sphere& sphere);

bool intersects(const Sphere& sphere, const AABB& aabb);

bool intersects(const OBB& obb, const Frustum& frustum);

bool intersects(const Frustum& frustum, const OBB& obb);

bool intersects(const OBB& obb, const Vector3& point);

bool intersects(const OBB& obb, const LineSegment& line_segment);

bool intersects(const LineSegment& line_segment, const OBB& obb);

bool intersects(const OBB& obb_1, const OBB& obb_2);

bool intersects(const OBB& obb, const Plane& plane);

bool intersects(const Plane& plane, const OBB& obb);

bool intersects(const OBB& obb, const Ray& ray);

bool intersects(const Ray& ray, const OBB& obb);

bool intersects(const OBB& obb, const Sphere& sphere);

bool intersects(const Sphere& sphere, const OBB& obb);

bool intersects(const Vector3& point, const OBB& obb);

bool intersects(const Sphere& sphere, const Frustum& frustum);

bool intersects(const Frustum& frustum, const Sphere& sphere);

bool intersects(const Sphere& sphere, const LineSegment& line_segment);

bool intersects(const LineSegment& line_segment, const Sphere& sphere);

bool intersects(const Sphere& sphere, const Plane& plane);

bool intersects(const Plane& plane, const Sphere& sphere);

bool intersects(const Sphere& sphere, const Ray& ray);

bool intersects(const Ray& ray, const Sphere& sphere);

bool intersects(const Sphere& sphere_1, const Sphere& sphere_2);

bool intersects(const Sphere& sphere, const Vector3& point);

bool intersects(const Vector3& point, const Sphere& sphere);

bool intersects(const Plane& plane, const Vector3& point);

bool intersects(const Vector3& point, const Plane& plane);

bool intersects(const Plane& plane, const LineSegment& line_segment);

bool intersects(const LineSegment& line_segment, const Plane& plane);

bool intersects(const Plane& plane_1, const Plane& plane_2);

bool intersects(const Plane& plane, const Ray& ray);

bool intersects(const Ray& ray, const Plane& plane);

bool intersects(const LineSegment& line_segment, const Vector3& point);

bool intersects(const Vector3& point, const LineSegment& line_segment);

bool intersects(const Ray& ray, const Vector3& point);

bool intersects(const Vector3& point, const Ray& ray);

}  // namespace ufomap_geometry

#endif  // UFOMAP_GEOMETRY_INTERSECTS_H