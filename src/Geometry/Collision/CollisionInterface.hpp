#pragma once

#include "BlockMatrix.h"
#include "EigenAll.h"
#include "Pattern.h"

CONCEPT_MODEL_IDIOM_BEGIN(CollisionInterface)
    ADD_INTERFACE_FUNCTION(int GetDOF() const, GetDOF())
    ADD_INTERFACE_FUNCTION(const BlockVector& GetCollisionVertexDerivative(int idx) const, GetCollisionVertexDerivative(idx))
    ADD_INTERFACE_FUNCTION(void ComputeCollisionVertex(const Ref<const VectorXd>& x) const, ComputeCollisionVertex(x))
    ADD_INTERFACE_FUNCTION(void ComputeCollisionVertexVelocity(const Ref<const VectorXd>& v) const, ComputeCollisionVertexVelocity(v))
    ADD_INTERFACE_FUNCTION(Vector3d GetCollisionVertexVelocity(int idx) const, GetCollisionVertexVelocity(idx))
    ADD_INTERFACE_FUNCTION(const MatrixXd& GetCollisionVertices() const, GetCollisionVertices())
    ADD_INTERFACE_FUNCTION(const MatrixXi& GetCollisionEdgeTopo() const, GetCollisionEdgeTopo())
    ADD_INTERFACE_FUNCTION(const MatrixXi& GetCollisionFaceTopo() const, GetCollisionFaceTopo())
CONCEPT_MODEL_IDIOM_CONCEPT
    ADD_CONCEPT_FUNCTION(int GetDOF() const)
    ADD_CONCEPT_FUNCTION(const BlockVector& GetCollisionVertexDerivative(int idx) const)
    ADD_CONCEPT_FUNCTION(void ComputeCollisionVertex(const Ref<const VectorXd>& x) const)
    ADD_CONCEPT_FUNCTION(void ComputeCollisionVertexVelocity(const Ref<const VectorXd>& v) const)
    ADD_CONCEPT_FUNCTION(Vector3d GetCollisionVertexVelocity(int idx) const)
    ADD_CONCEPT_FUNCTION(const MatrixXd& GetCollisionVertices() const)
    ADD_CONCEPT_FUNCTION(const MatrixXi& GetCollisionEdgeTopo() const)
    ADD_CONCEPT_FUNCTION(const MatrixXi& GetCollisionFaceTopo() const)
CONCEPT_MODEL_IDIOM_MODEL
    ADD_MODEL_FUNCTION(int GetDOF() const, GetDOF())
    ADD_MODEL_FUNCTION(const BlockVector& GetCollisionVertexDerivative(int idx) const, GetCollisionVertexDerivative(idx))
    ADD_MODEL_FUNCTION(void ComputeCollisionVertex(const Ref<const VectorXd>& x) const, ComputeCollisionVertex(x))
    ADD_MODEL_FUNCTION(void ComputeCollisionVertexVelocity(const Ref<const VectorXd>& v) const, ComputeCollisionVertexVelocity(v))
    ADD_MODEL_FUNCTION(Vector3d GetCollisionVertexVelocity(int idx) const, GetCollisionVertexVelocity(idx))
    ADD_MODEL_FUNCTION(const MatrixXd& GetCollisionVertices() const, GetCollisionVertices())
    ADD_MODEL_FUNCTION(const MatrixXi& GetCollisionEdgeTopo() const, GetCollisionEdgeTopo())
    ADD_MODEL_FUNCTION(const MatrixXi& GetCollisionFaceTopo() const, GetCollisionFaceTopo())
CONCEPT_MODEL_IDIOM_END