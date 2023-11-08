#pragma once

#include "EigenAll.h"

namespace IPC {
	double edge_edge_distance_unclassified(
		const Vector3d& ea0,
		const Vector3d& ea1,
		const Vector3d& eb0,
		const Vector3d& eb1
	);

	double point_triangle_distance_unclassified(
		const Vector3d& p,
		const Vector3d& t0,
		const Vector3d& t1,
		const Vector3d& t2
	);
}