#ifndef UFOMAP_GEOMETRY_LINE_SEGMENT_H
#define UFOMAP_GEOMETRY_LINE_SEGMENT_H

#include <ufomap/math/vector3.h>

using namespace ufomap_math;

namespace ufomap_geometry
{
struct LineSegment
{
	Vector3 start;
	Vector3 end;

	inline LineSegment()
	{
	}

	inline LineSegment(const LineSegment& line_segment)
		: start(line_segment.start), end(line_segment.end)
	{
	}

	inline LineSegment(const Vector3& start, const Vector3& end) : start(start), end(end)
	{
	}

	inline void translate(const Vector3& translation)
	{
		// TODO: Implement
	}

	inline void rotate(const Vector3& rotation)
	{
		// TODO: Implement
	}

	inline void transform(const Pose6& transform)
	{
		// TODO: Implement
	}
};
}  // namespace ufomap_geometry

#endif  // UFOMAP_GEOMETRY_LINE_SEGMENT_H