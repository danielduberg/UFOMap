#ifndef UFOMAP_GEOMETRY_TYPES_H
#define UFOMAP_GEOMETRY_TYPES_H

#include <ufomap/geometry/aabb.h>
#include <ufomap/geometry/frustum.h>
#include <ufomap/geometry/line_segment.h>
#include <ufomap/geometry/obb.h>
#include <ufomap/geometry/plane.h>
#include <ufomap/geometry/ray.h>
#include <ufomap/geometry/sphere.h>

#include <variant>

namespace ufomap_geometry
{
using BoundingVar = std::variant<AABB, Frustum, LineSegment, OBB, Plane, Ray, Sphere>;

}  // namespace ufomap_geometry

#endif  // UFOMAP_GEOMETRY_TYPES_H