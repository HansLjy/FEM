#include "UnclassifiedDistance.hpp"
#include "DistanceType.h"
#include "PointEdgeDistance.h"
#include "PointPointDistance.h"
#include "EdgeEdgeDistance.h"
#include "PointTriangleDistance.h"

double IPC::edge_edge_distance_unclassified(
	const Vector3d& ea0,
	const Vector3d& ea1,
	const Vector3d& eb0,
	const Vector3d& eb1
) {
	switch (edge_edge_distance_type(ea0, ea1, eb0, eb1)) {
	case 0:
		return point_point_distance(ea0, eb0);
	case 1:
		return point_point_distance(ea0, eb1);
	case 2:
		return point_edge_distance(ea0, eb0, eb1);
	case 3:
		return point_point_distance(ea1, eb0);
	case 4:
		return point_point_distance(ea1, eb1);
	case 5:
		return point_edge_distance(ea1, eb0, eb1);
	case 6:
		return point_edge_distance(eb0, ea0, ea1);
	case 7:
		return point_edge_distance(eb1, ea0, ea1);
	case 8:
		return edge_edge_distance(ea0, ea1, eb0, eb1);
	default:
		return std::numeric_limits<double>::max();
	}
}

double IPC::point_triangle_distance_unclassified(
	const Vector3d& p,
	const Vector3d& t0,
	const Vector3d& t1,
	const Vector3d& t2
) {
	switch (point_triangle_distance_type(p, t0, t1, t2)) {
	case 0:
		return point_point_distance(p, t0);
	case 1:
		return point_point_distance(p, t1);
	case 2:
		return point_point_distance(p, t2);
	case 3:
		return point_edge_distance(p, t0, t1);
	case 4:
		return point_edge_distance(p, t1, t2);
	case 5:
		return point_edge_distance(p, t2, t0);
	case 6:
		return point_triangle_distance(p, t0, t1, t2);
	default:
		return std::numeric_limits<double>::max();
	}
}