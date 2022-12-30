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
	virtual void Bind(const Object& obj) = 0;
    virtual void ComputeCollisionShape(const Ref<const VectorXd>& x) = 0;

	/**
	 * @return Projection matrix P of surface vertices. P is defined to 
	 *		   satisfy the equation Px = surface vertices
	 * @note However, the relation between surface vertices and coordinates
	 *		 may not be linear, in such cases, P is defined to be the first
	 * 		 derivative of whatever bizarre relationship between them.
	 */
	const SparseMatrixXd& GetVertexProjectionMatrix() const {return _vertex_projections;}
    const MatrixXd& GetCollisionVertices() const {return _vertices;}
    const MatrixXi& GetCollisionEdgeTopo() const {return _edge_topo;}
    const MatrixXi& GetCollisionFaceTopo() const {return _face_topo;}

	virtual ~CollisionShape() = default;
	CollisionShape() = default;
	CollisionShape(const CollisionShape& rhs) = delete;

protected:
    MatrixXd _vertices;
	SparseMatrixXd _vertex_projections;
    MatrixXi _edge_topo;
    MatrixXi _face_topo;
};

class NullCollisionShape : public CollisionShape {
public:
	void ComputeCollisionShape(const Ref<const VectorXd> &x) override;
};

#endif //FEM_COLLISIONSHAPE_H
