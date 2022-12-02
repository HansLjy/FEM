//
// Created by hansljy on 11/30/22.
//

#ifndef FEM_HASHCULLING_H
#define FEM_HASHCULLING_H

#include "CollisionCulling.h"
#include "SpatialHashing/SpatialHashing.hpp"

struct EdgePrimitiveInfo {
    int _obj_id;
    int _primitive_id;
    Vector3d _vertex1;
    Vector3d _vertex2;
};

struct VertexPrimitiveInfo {
    int _obj_id;
    int _primitive_id;
    Vector3d _vertex;
};

class HashCulling : public CollisionCulling {
public:
    HashCulling(double grid_size, unsigned int hash_table_size) : _edge_hash_table(grid_size, hash_table_size),
                                                                  _vertice_hash_table(grid_size, hash_table_size) {}
    void ComputeConstraintSet(const Ref<const Eigen::VectorXd> &x, const std::shared_ptr<const ObjectIterator> &begin, int time_stamp, double d, std::vector<CollisionInfo> &info) override;

    DERIVED_DECLARE_CLONE(CollisionCulling)
protected:
    SpatialHashing<EdgePrimitiveInfo> _edge_hash_table;
    SpatialHashing<VertexPrimitiveInfo> _vertice_hash_table;
};

#endif //FEM_HASHCULLING_H
