#include "TOIEstimator.hpp"
#include "Collision/CollisionUtil/ProcessPrimitivePair.hpp"

double TOIEstimator::GetTOI(
	const std::vector<PrimitivePair> &constraint_set,
	const std::vector<CollisionInterface> &objs
) const {
	double toi = 1;

	PROCESS_PRIMITIVE_PAIR(
		constraint_set, objs,
		
		double local_toi = _ccd->VertexFaceCollision(vertex, face1, face2, face3, vertex_velocity, face_velocity1, face_velocity2, face_velocity3);
		if (local_toi < toi) {
			toi = local_toi;
		},

		double local_toi = _ccd->EdgeEdgeCollision(edge11, edge12, edge21, edge22, edge_velocity11, edge_velocity12, edge_velocity21, edge_velocity22);

		if (local_toi < toi) {
			toi = local_toi;
		}
		,
	)

	return toi;
}

double TOIEstimator::GetLocalTOIs(
	const std::vector<PrimitivePair> &constraint_set,
	const std::vector<CollisionInterface> &objs,
	std::vector<double> &local_tois
) const {
	local_tois.reserve(local_tois.size() + constraint_set.size());
	double toi = 1;

	PROCESS_PRIMITIVE_PAIR(
		constraint_set, objs,
		
		double local_toi = _ccd->VertexFaceCollision(vertex, face1, face2, face3, vertex_velocity, face_velocity1, face_velocity2, face_velocity3);
		local_tois.emplace_back(local_toi);
		if (local_toi < toi) {
			toi = local_toi;
		},

		double local_toi = _ccd->EdgeEdgeCollision(edge11, edge12, edge21, edge22, edge_velocity11, edge_velocity12, edge_velocity21, edge_velocity22);
		local_tois.emplace_back(local_toi);
		if (local_toi < toi) {
			toi = local_toi;
		},
	);
	return toi;
}