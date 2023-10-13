#pragma once

#include "Pattern.h"
#include "Object.hpp"
#include "Collision/CollisionInterface.hpp"

CONCEPT_MODEL_IDIOM_BEGIN(Collider)	
	ADD_INTERFACE_FUNCTION(bool Intersect(const Vector3d& x) const, Intersect(x))
	ADD_INTERFACE_FUNCTION(Vector3d SurfaceProject(const Vector3d& point) const, SurfaceProject(point))
CONCEPT_MODEL_IDIOM_CONCEPT
	ADD_CONCEPT_FUNCTION(bool Intersect(const Vector3d& x) const)
	ADD_CONCEPT_FUNCTION(Vector3d SurfaceProject(const Vector3d& point) const)
CONCEPT_MODEL_IDIOM_MODEL
	ADD_MODEL_FUNCTION(bool Intersect(const Vector3d& x) const, Intersect(x))
	ADD_MODEL_FUNCTION(Vector3d SurfaceProject(const Vector3d& point) const, SurfaceProject(point))
CONCEPT_MODEL_IDIOM_END

class PDCollisionHandler {
public:
	PDCollisionHandler(double collision_stiffness) : _collision_stiffness(collision_stiffness) {}

	void BindObjects(
		const typename std::vector<Object>::const_iterator &begin,
		const typename std::vector<Object>::const_iterator &end
	);

	void BindColliders(
		const typename std::vector<Object>::const_iterator &begin,
		const typename std::vector<Object>::const_iterator &end
	);

	void ComputeConstraintSet(const Ref<const VectorXd>& x);

	void GetGlobalMatrix(COO& coo, int x_offset, int y_offset) const;
	void LocalProject(const Ref<const VectorXd>& x, Ref<VectorXd> y, int offset) const;

protected:
	struct PenetratedVertex {
		int _obj_id;
		int _vertex_id;
		Vector3d _normal;
		double _offset;
	};

	double _collision_stiffness;

	std::vector<int> _offsets;

	std::vector<PenetratedVertex> _penetrated_vertices;

	InterfaceContainer<CollisionInterface> _obj_container;
	InterfaceContainer<Collider> _collider_container;	
};