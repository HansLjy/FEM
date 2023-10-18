#include "SpatialHashingCulling.hpp"

SpatialHashingCulling::SpatialHashingCulling(const json& config):
	_edge_hash_table(config["grid-length"], config["hash-table-size"]),
	_vertex_hash_table(config["grid-length"], config["hash-table-size"]) {
}

void SpatialHashingCulling::GetCCDSet(
	const std::vector<CollisionInterface> &objs,
	const std::vector<int> &offsets,
	std::vector<PrimitivePair> &ccd_set) {

	ccd_set.clear();
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
			Vector3d vertice_next = (vertices.row(i).transpose() + obj.GetCollisionVertexVelocity(i));

			Vector3d min_point = vertice.cwiseMin(vertice_next);
			Vector3d max_point = vertice.cwiseMax(vertice_next);

            _vertex_hash_table.Insert(
                min_point, max_point,
                VertexPrimitiveInfo {obj_id, i, vertice},
                _time_stamp
            );
        }

        for (int i = 0; i < num_edges; i++) {
            Vector3d edge_vertex1 = vertices.row(edge_topo(i, 0)).transpose();
			Vector3d edge_vertex_next1 = vertices.row(edge_topo(i, 0)).transpose() + obj.GetCollisionVertexVelocity(edge_topo(i, 0));
            Vector3d edge_vertex2 = vertices.row(edge_topo(i, 1)).transpose();
			Vector3d edge_vertex_next2 = vertices.row(edge_topo(i, 1)).transpose() + obj.GetCollisionVertexVelocity(edge_topo(i, 1));
            Vector3d bb_min = edge_vertex1.cwiseMin(edge_vertex_next1).cwiseMin(edge_vertex2.cwiseMin(edge_vertex_next2));
            Vector3d bb_max = edge_vertex1.cwiseMax(edge_vertex_next1).cwiseMax(edge_vertex2.cwiseMax(edge_vertex_next2));

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
        for (int face_id = 0; face_id < num_faces; face_id++) {
            const auto topo = face_topo.row(face_id);
            Vector3d face_vertex1 = vertices.row(topo(0)).transpose();
			Vector3d face_vertex_v1 = obj.GetCollisionVertexVelocity(topo(0));
            Vector3d face_vertex_next1 = face_vertex1 + face_vertex_v1;
            Vector3d face_vertex2 = vertices.row(topo(1)).transpose();
			Vector3d face_vertex_v2 = obj.GetCollisionVertexVelocity(topo(1));
            Vector3d face_vertex_next2 = face_vertex2 + face_vertex_v2;
            Vector3d face_vertex3 = vertices.row(topo(2)).transpose();
			Vector3d face_vertex_v3 = obj.GetCollisionVertexVelocity(topo(2));
            Vector3d face_vertex_next3 = face_vertex3 + face_vertex_v3;

            Vector3d bb_min = face_vertex1.cwiseMin(face_vertex2.cwiseMin(face_vertex3)).cwiseMin(face_vertex_next1.cwiseMin(face_vertex_next2.cwiseMin(face_vertex_next3)));
            Vector3d bb_max = face_vertex1.cwiseMax(face_vertex2.cwiseMax(face_vertex3)).cwiseMax(face_vertex_next1.cwiseMax(face_vertex_next2.cwiseMax(face_vertex_next3)));

            auto candidates = _vertex_hash_table.Find(bb_min, bb_max, _time_stamp);
            for (const auto& candidate : candidates) {
				if (candidate._obj_id == obj_id && (candidate._primitive_id == topo(0) || candidate._primitive_id == topo(1) || candidate._primitive_id == topo(2))) {
					continue;
				}
				ccd_set.push_back(PrimitivePair{
					CollisionType::kVertexFace,
					candidate._obj_id, obj_id,
					candidate._primitive_id, face_id
				});
            }
        }

        /* edge edge collision */
        for (int edge_id = 0; edge_id < num_edges; edge_id++) {
            const RowVector2i indices = edge_topo.row(edge_id);

            Vector3d edge_vertex1 = vertices.row(indices(0)).transpose();
			Vector3d edge_vertex_v1 = obj.GetCollisionVertexVelocity(indices(0));
			Vector3d edge_vertex_next1 = edge_vertex1 + edge_vertex_v1;
            Vector3d edge_vertex2 = vertices.row(indices(1)).transpose();
			Vector3d edge_vertex_v2 = obj.GetCollisionVertexVelocity(indices(1));
			Vector3d edge_vertex_next2 = edge_vertex2 + edge_vertex_v2;
			
            Vector3d bb_min = edge_vertex1.cwiseMin(edge_vertex_next1).cwiseMin(edge_vertex2.cwiseMin(edge_vertex_next2));
            Vector3d bb_max = edge_vertex1.cwiseMax(edge_vertex_next1).cwiseMax(edge_vertex2.cwiseMax(edge_vertex_next2));

            auto candidates = _edge_hash_table.Find(bb_min, bb_max, _time_stamp);
            for (const auto& candidate : candidates) {
				if (candidate._obj_id > obj_id) {
					continue;
				}
				if (candidate._obj_id == obj_id) {
					RowVector2i other_topo = objs[candidate._obj_id].GetCollisionEdgeTopo().row(candidate._primitive_id);
					if (other_topo(0) == indices(0) || other_topo(0) == indices(1) || other_topo(1) == indices(0) || other_topo(1) == indices(1)) {
						continue;
					}
				}
				ccd_set.push_back(PrimitivePair{
					CollisionType::kEdgeEdge,
					candidate._obj_id, obj_id,
					candidate._primitive_id, edge_id
				});
            }
        }
		obj_id++;
    }
}