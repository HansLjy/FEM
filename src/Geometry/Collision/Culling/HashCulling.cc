//
// Created by hansljy on 11/30/22.
//

#include "HashCulling.h"
#include "SpatialHashing/SpatialHashing.hpp"
#include "Collision/CollisionUtility.h"
#include "Object.h"

void HashCulling::ComputeConstraintSet(const Ref<const VectorXd> &x, const std::vector<Object *> &objs, int time_stamp, double d, std::vector<CollisionInfo> &info) {
    /* insert into hash table */
    int cur_offset = 0;
	for (auto obj : objs) {
		obj->ComputeCollisionShape(x.segment(cur_offset, obj->GetDOF()));
		cur_offset += obj->GetDOF();
	}

	cur_offset = 0;
    int obj_id = 0;
    for (auto obj : objs) {
        const Matrix3d rotation = obj->GetFrameRotation();
        const Vector3d translation = obj->GetFrameX();

		const auto& vertices = obj->GetCollisionVertices();
		const auto& face_topo = obj->GetCollisionFaceTopo();
		const auto& edge_topo = obj->GetCollisionEdgeTopo();
        const int num_vertices = vertices.rows();
        const int num_faces = face_topo.rows();
        const int num_edges = edge_topo.rows();

        for (int i = 0; i < num_vertices; i++) {
            Vector3d vertice = rotation * vertices.row(i).transpose() + translation;
            _vertice_hash_table.Insert(
                vertice,
                VertexPrimitiveInfo {obj_id, i, vertice},
                time_stamp
            );
        }

        for (int i = 0; i < num_edges; i++) {
            Vector3d edge_vertex1 = rotation * vertices.row(edge_topo(i, 0)).transpose() + translation;
            Vector3d edge_vertex2 = rotation * vertices.row(edge_topo(i, 1)).transpose() + translation;
            Vector3d bb_min = edge_vertex1.cwiseMin(edge_vertex2);
            Vector3d bb_max = edge_vertex1.cwiseMax(edge_vertex2);

            _edge_hash_table.Insert(
                bb_min, bb_max,
                EdgePrimitiveInfo {obj_id, i, edge_vertex1, edge_vertex2},
                time_stamp
            );
        }
        cur_offset += obj->GetDOF();
        obj_id++;
    }

    /* find in hash table */

    cur_offset = 0;
    obj_id = 0;
    for (auto obj : objs) {
        const Matrix3d rotation = obj->GetFrameRotation();
        const Vector3d translation = obj->GetFrameX();

		const auto& vertices = obj->GetCollisionVertices();
		const auto& face_topo = obj->GetCollisionFaceTopo();
		const auto& edge_topo = obj->GetCollisionEdgeTopo();

        const int num_faces = face_topo.rows();
        const int num_edges = edge_topo.rows();

        /* vertex-face collision */
        for (int i = 0; i < num_faces; i++) {
            const auto topo = face_topo.row(i);
            Vector3d face_vertex1 = rotation * vertices.row(topo(0)).transpose() + translation;
            Vector3d face_vertex2 = rotation * vertices.row(topo(1)).transpose() + translation;
            Vector3d face_vertex3 = rotation * vertices.row(topo(2)).transpose() + translation;

            Vector3d bb_min = face_vertex1.cwiseMin(face_vertex2.cwiseMin(face_vertex3)) - Vector3d::Constant(d);
            Vector3d bb_max = face_vertex1.cwiseMax(face_vertex2.cwiseMax(face_vertex3)) + Vector3d::Constant(d);

            auto candidates = _vertice_hash_table.Find(bb_min, bb_max, time_stamp);
            for (const auto& candidate : candidates) {
                if (GetVFDistance(candidate._vertex, face_vertex1, face_vertex2, face_vertex3) < d) {
                    info.push_back(CollisionInfo{
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

            Vector3d edge_vertex1 = rotation * vertices.row(topo(0)).transpose() + translation;
            Vector3d edge_vertex2 = rotation * vertices.row(topo(1)).transpose() + translation;

            Vector3d bb_min = edge_vertex1.cwiseMin(edge_vertex2);
            Vector3d bb_max = edge_vertex1.cwiseMax(edge_vertex2);

            auto candidates = _edge_hash_table.Find(bb_min, bb_max, time_stamp);
            for (const auto& candidate : candidates) {
                if (GetEEDistance(candidate._vertex1, candidate._vertex2, edge_vertex1, edge_vertex2) < d) {
                    info.push_back(CollisionInfo{
                        CollisionType::kEdgeEdge,
                        candidate._obj_id, obj_id,
                        candidate._primitive_id, i
                    });
                }
            }
        }
    }
}