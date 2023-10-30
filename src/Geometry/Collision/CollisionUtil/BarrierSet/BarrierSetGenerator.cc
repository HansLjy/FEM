#include "BarrierSetGenerator.hpp"
#include "Collision/CollisionUtility.h"
#include "Pattern.h"

template<>
Factory<BarrierSetGenerator>* Factory<BarrierSetGenerator>::_the_factory = nullptr;

const bool shbg_registered = FactoryRegistration::RegisterForFactory<BarrierSetGenerator, SpatialHashingBarrierSetGenerator>("spatial-hashing");

BarrierSetGenerator* BarrierSetGenerator::GetProductFromConfig(const json &config) {
	return Factory<BarrierSetGenerator>::GetInstance()->GetProduct(config["type"], config);
}

SpatialHashingBarrierSetGenerator* SpatialHashingBarrierSetGenerator::CreateFromConfig(const json &config) {
	return new SpatialHashingBarrierSetGenerator(config["d-hat"], config["grid-length"], config["hash-table-size"]);
}

void SpatialHashingBarrierSetGenerator::GenerateBarrierSet(
	const std::vector<CollisionInterface> &objs,
	const std::vector<int> &offsets,
	std::vector<PrimitivePair> &barrier_set) {

	_time_stamp++;

    int obj_id = 0;
    for (auto obj : objs) {
		const auto& vertices = obj.GetCollisionVertices();
		const auto& face_topo = obj.GetCollisionFaceTopo();
		const auto& edge_topo = obj.GetCollisionEdgeTopo();
        const int num_vertices = vertices.rows();
        const int num_faces = face_topo.rows();
        const int num_edges = edge_topo.rows();

        for (int i = 0; i < num_vertices; i++) {
            Vector3d vertice = vertices.row(i).transpose();
            _vertex_hash_table.Insert(
                vertice,
                VertexPrimitiveInfo {obj_id, i, vertice},
                _time_stamp
            );
        }

        for (int i = 0; i < num_edges; i++) {
            Vector3d edge_vertex1 = vertices.row(edge_topo(i, 0)).transpose();
            Vector3d edge_vertex2 = vertices.row(edge_topo(i, 1)).transpose();
            Vector3d bb_min = edge_vertex1.cwiseMin(edge_vertex2);
            Vector3d bb_max = edge_vertex1.cwiseMax(edge_vertex2);

            _edge_hash_table.Insert(
                bb_min, bb_max,
                EdgePrimitiveInfo {obj_id, i, edge_vertex1, edge_vertex2},
                _time_stamp
            );
        }
        obj_id++;
    }

    /* find in hash table */

    obj_id = 0;
    for (auto obj : objs) {
		const auto& vertices = obj.GetCollisionVertices();
		const auto& face_topo = obj.GetCollisionFaceTopo();
		const auto& edge_topo = obj.GetCollisionEdgeTopo();

        const int num_faces = face_topo.rows();
        const int num_edges = edge_topo.rows();

        /* vertex-face collision */
        for (int i = 0; i < num_faces; i++) {
            const auto topo = face_topo.row(i);
            Vector3d face_vertex1 = vertices.row(topo(0)).transpose();
            Vector3d face_vertex2 = vertices.row(topo(1)).transpose();
            Vector3d face_vertex3 = vertices.row(topo(2)).transpose();

            Vector3d bb_min = face_vertex1.cwiseMin(face_vertex2.cwiseMin(face_vertex3)) - Vector3d::Constant(_d_hat);
            Vector3d bb_max = face_vertex1.cwiseMax(face_vertex2.cwiseMax(face_vertex3)) + Vector3d::Constant(_d_hat);

            auto candidates = _vertex_hash_table.Find(bb_min, bb_max, _time_stamp);
            for (const auto& candidate : candidates) {
				if (candidate._obj_id == obj_id && (candidate._primitive_id == topo(0) || candidate._primitive_id == topo(1) || candidate._primitive_id == topo(2))) {
					continue;
				}
				auto distance = GetVFDistance(candidate._vertex, face_vertex1, face_vertex2, face_vertex3);
                if (distance < _d_hat) {
                    barrier_set.push_back(PrimitivePair{
                        CollisionType::kVertexFace,
                        candidate._obj_id, obj_id,
                        candidate._primitive_id, i
                    });
                }
            }
        }

        /* edge edge collision */
        for (int i = 0; i < num_edges; i++) {
            const auto topo = edge_topo.row(i);

            Vector3d edge_vertex1 = vertices.row(topo(0)).transpose();
            Vector3d edge_vertex2 = vertices.row(topo(1)).transpose();

            Vector3d bb_min = edge_vertex1.cwiseMin(edge_vertex2) - Vector3d::Constant(_d_hat);
            Vector3d bb_max = edge_vertex1.cwiseMax(edge_vertex2) + Vector3d::Constant(_d_hat);

            auto candidates = _edge_hash_table.Find(bb_min, bb_max, _time_stamp);
            for (const auto& candidate : candidates) {
				if (candidate._obj_id > obj_id) {
					continue;
				}
				if (candidate._obj_id == obj_id) {
					RowVector2i other_topo = objs[candidate._obj_id].GetCollisionEdgeTopo().row(candidate._primitive_id);
					if (other_topo(0) == topo(0) || other_topo(0) == topo(1) || other_topo(1) == topo(0) || other_topo(1) == topo(1)) {
						continue;
					}
				}
				auto distance = GetEEDistance(candidate._vertex1, candidate._vertex2, edge_vertex1, edge_vertex2);
                if (distance < _d_hat) {
                    barrier_set.push_back(PrimitivePair{
                        CollisionType::kEdgeEdge,
                        candidate._obj_id, obj_id,
                        candidate._primitive_id, i
                    });
                }
            }
        }
		obj_id++;
    }

}