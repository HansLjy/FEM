//
// Created by hansljy on 12/21/22.
//

#ifndef FEM_COLLISIONSHAPE_H
#define FEM_COLLISIONSHAPE_H

#include "EigenAll.h"
#include "Pattern.h"

class Object;

class CollisionShape {
public:
    virtual void ComputeCollisionShape(const Object& obj, const Ref<const VectorXd>& x) = 0;
    const MatrixXd& GetCollisionVertices() const {return _vertices;}
    const MatrixXi& GetCollisionEdgeTopo() const {return _edge_topo;}
    const MatrixXi& GetCollisionFaceTopo() const {return _face_topo;}

	virtual ~CollisionShape() = default;

	BASE_DECLARE_CLONE(CollisionShape)

protected:
    MatrixXd _vertices;
    MatrixXi _edge_topo;
    MatrixXi _face_topo;
};

#endif //FEM_COLLISIONSHAPE_H
