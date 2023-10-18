#pragma once

#include "JsonUtil.h"
#include "EigenAll.h"
#include "BlockMatrix.h"

template<class CollisionShape, class Derived>
class CollisionShapeAdapter : public CollisionShape {
public:
	void Initialize();
    CollisionShapeAdapter(CollisionShape&& rhs) : CollisionShape(std::move(rhs)) {}
	void ComputeCollisionVertex(const Ref<const VectorXd>& x);
	void ComputeCollisionVertexVelocity(const Ref<const VectorXd>& v);
    Vector3d GetCollisionVertexVelocity(int idx) const;
	double GetCollisionVertexMass(int idx) const;
	const BlockVector& GetCollisionVertexDerivative(int idx) const;
	const MatrixXd& GetCollisionVertices() const;
	const MatrixXi& GetCollisionEdgeTopo() const;
	const MatrixXi& GetCollisionFaceTopo() const;
};

template<class CollisionShape, class Derived>
void CollisionShapeAdapter<CollisionShape, Derived>::Initialize() {
	CollisionShape::Initialize(static_cast<Derived*>(this));
}
template<class CollisionShape, class Derived>
void CollisionShapeAdapter<CollisionShape, Derived>::ComputeCollisionVertex(const Ref<const VectorXd> &x) {
    CollisionShape::ComputeCollisionVertex(static_cast<Derived*>(this), x);
}


template<class CollisionShape, class Derived>
void CollisionShapeAdapter<CollisionShape, Derived>::ComputeCollisionVertexVelocity(const Ref<const VectorXd>& v) {
    CollisionShape::ComputeCollisionVertexVelocity(static_cast<Derived*>(this), v);
}

template<class CollisionShape, class Derived>
Vector3d CollisionShapeAdapter<CollisionShape, Derived>::GetCollisionVertexVelocity(int idx) const {
    return CollisionShape::GetCollisionVertexVelocity(static_cast<const Derived*>(this), idx);
}

template<class CollisionShape, class Derived>
double CollisionShapeAdapter<CollisionShape, Derived>::GetCollisionVertexMass(int idx) const {
	return CollisionShape::GetCollisionVertexMass(static_cast<const Derived*>(this), idx);
}


template<class CollisionShape, class Derived>
const BlockVector& CollisionShapeAdapter<CollisionShape, Derived>::GetCollisionVertexDerivative(int idx) const {
    return CollisionShape::GetCollisionVertexDerivative(static_cast<const Derived*>(this), idx);
}

template<class CollisionShape, class Derived>
const MatrixXd& CollisionShapeAdapter<CollisionShape, Derived>::GetCollisionVertices() const {
    return CollisionShape::GetCollisionVertices(static_cast<const Derived*>(this));
}

template<class CollisionShape, class Derived>
const MatrixXi& CollisionShapeAdapter<CollisionShape, Derived>::GetCollisionEdgeTopo() const {
    return CollisionShape::GetCollisionEdgeTopo(static_cast<const Derived*>(this));
}

template<class CollisionShape, class Derived>
const MatrixXi& CollisionShapeAdapter<CollisionShape, Derived>::GetCollisionFaceTopo() const {
    return CollisionShape::GetCollisionFaceTopo(static_cast<const Derived*>(this));
}