//
// Created by hansljy on 11/23/22.
//

#ifndef FEM_COLLISIONCULLING_H
#define FEM_COLLISIONCULLING_H

#include "EigenAll.h"
#include "Pattern.h"

class Object;

enum struct CollisionType {
    kEdgeEdge,
    kVertexFace
};
struct CollisionInfo {
    CollisionType _type;
    int _obj_id1, _obj_id2;
    int _primitive_id1, _primitive_id2;
};

class CollisionCulling {
public:
    virtual void ComputeConstraintSet(const Ref<const VectorXd> &x, const std::vector<Object*>& objs , int time_stamp, double d,
                                      std::vector<CollisionInfo> &info) = 0;

    virtual ~CollisionCulling() = default;
};

DECLARE_XXX_FACTORY(CollisionCulling)

#endif //FEM_COLLISIONCULLING_H
