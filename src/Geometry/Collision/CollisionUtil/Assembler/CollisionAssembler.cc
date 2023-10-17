#include "CollisionAssembler.hpp"

void CollisionAssembler::ComputeCollisionVertex(
	const Ref<const VectorXd> &x,
	std::vector<CollisionInterface> &objs
) const {
	int offset = 0;
	for (auto& obj : objs) {
		obj.ComputeCollisionVertex(x.segment(offset, obj.GetDOF()));
		offset += obj.GetDOF();
	}
}

void CollisionAssembler::ComputeCollisionVertexVelocity(
	const Ref<const VectorXd> &v,
	std::vector<CollisionInterface> &objs
) const {
	int offset = 0;
	for (auto& obj : objs) {
		obj.ComputeCollisionVertexVelocity(v.segment(offset, obj.GetDOF()));
		offset += obj.GetDOF();
	}
}