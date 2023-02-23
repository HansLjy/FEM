//
// Created by hansljy on 12/21/22.
//

#ifndef FEM_COLLISIONSHAPE_H
#define FEM_COLLISIONSHAPE_H

#include "EigenAll.h"
#include "Pattern.h"
#include "Object.h"
#include "BlockMatrix.h"

class Object;

class CollisionShape {
public:
	virtual void Bind(const Object& obj) = 0;
    virtual void ComputeCollisionShape(const Ref<const VectorXd>& x) = 0;

	virtual const BlockVector& GetVertexDerivative(int idx) const = 0;

	virtual Vector3d GetCollisionVertexVelocity(const Ref<const VectorXd>& v, int idx) const = 0;
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

class SampledCollisionShape : public CollisionShape {
public:
	void Bind(const Object &obj) override;
	void ComputeCollisionShape(const Ref<const VectorXd> &x) override;
	Vector3d GetCollisionVertexVelocity(const Ref<const VectorXd> &v, int idx) const override;
	const BlockVector& GetVertexDerivative(int idx) const override;

protected:
	std::vector<BlockVector> _vertex_derivatives;
};

class DecomposedCollisionShape : public CollisionShape {
public:
	void Bind(const Object &obj) override {
		// TODO:
	}
	void ComputeCollisionShape(const Ref<const VectorXd> &x) override {
		// TODO:
	}

	Vector3d GetCollisionVertexVelocity(const Ref<const VectorXd> &v, int idx) const override {
		// TODO:
	}

	const BlockVector & GetVertexDerivative(int idx) const override {
		// TODO:
	}
};

class NullCollisionShape : public CollisionShape {
public:
	void Bind(const Object &obj) override {}
	void ComputeCollisionShape(const Ref<const VectorXd> &x) override {}
	Vector3d GetCollisionVertexVelocity(const Ref<const VectorXd> &p, int idx) const override {return Vector3d::Zero();}
	const BlockVector & GetVertexDerivative(int idx) const override {return null_vector;}

protected:
	const BlockVector null_vector{0, 0, {}, {}, MatrixXd(0, 3)};
};

class FixedCollisionShape : public CollisionShape {
public:
	FixedCollisionShape(const MatrixXd& vertices, const SparseMatrixXd& projections, const MatrixXi& edge_topo, const MatrixXi& face_topo) {
		_vertices = vertices;
		_vertex_projections = projections;
		_edge_topo = edge_topo;
		_face_topo = face_topo;
	}
	Vector3d GetCollisionVertexVelocity(const Ref<const VectorXd> &p, int idx) const override {return Vector3d::Zero();}
	void Bind(const Object &obj) override {}
	void ComputeCollisionShape(const Ref<const VectorXd> &x) override {}

	const BlockVector & GetVertexDerivative(int idx) const override {return null_vector;}

protected:
	const BlockVector null_vector{0, 0, {}, {}, MatrixXd(0, 3)};
};

DECLARE_XXX_FACTORY(FixedCollisionShape)

#endif //FEM_COLLISIONSHAPE_H
