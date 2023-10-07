#pragma once

#include "ReducedObject.hpp"
#include "SampledObject.hpp"
#include "Render/TreeTrunkShape.hpp"
#include "ExternalForce/ExternalForceContainer.hpp"
#include "Collision/CollisionShape/CollisionShape.hpp"

class TreeTrunk : public SampledObject, public TreeTrunkShape, public NullCollisionShape, public ExternalForceContainer<TreeTrunk> {
public:
    TreeTrunk(double rho, double youngs_module, double radius_max, double radius_min, const VectorXd &x, const Vector3d &root);

    void Initialize() override {
        TreeTrunkShape::PreCompute(this);
        NullCollisionShape::Precompute(this);
    }

    double GetPotential(const Ref<const VectorXd>& x) const override;
    VectorXd GetPotentialGradient(const Ref<const VectorXd>& x) const override;
    void GetPotentialHessian(const Ref<const VectorXd>& x, COO& coo, int x_offset, int y_offset) const override;

    PROXY_EXTERNAL_FORCES_TO_CONTAINER(TreeTrunk)
    PROXY_RENDER_SHAPE(TreeTrunkShape)
    PROXY_COLLISION_SHAPE(NullCollisionShape)

    friend SampledObjectGravity<TreeTrunk>;

protected:
    const Vector3d _root;
    VectorXd _stiffness;
    VectorXd _alpha;
    VectorXd _x_rest;           // rest shape of the curve
    VectorXd _rest_length;      // length of every edge in the rest shape
    VectorXd _voronoi_length;   // length under the government of one point

private:
    static VectorXd GenerateMass(const VectorXd& x, double rho, double radius_max, double radius_min);
	static MatrixXi GenerateTopo(int n);
};

class ReducedTreeTrunk : public ReducedObject, public ProxyRenderShape, public NullCollisionShape {
public:
    explicit ReducedTreeTrunk(const json& config);
    ReducedTreeTrunk(int num_segments, double rho, double youngs_module, double radius_max, double radius_min,
                     const Vector3d &root, const VectorXd &control_points);
    void Initialize() override {
        ReducedObject::Initialize();
        NullCollisionShape::Precompute(this);
    }

    PROXY_RENDER_SHAPE(ProxyRenderShape)
    PROXY_COLLISION_SHAPE(NullCollisionShape)

    friend class DecomposedTreeTrunk;
    friend class AffineDecomposedTreeTrunk;

protected:
    const Vector3d _fixed_point;
    static VectorXd GenerateX(const VectorXd &control_points, int num_segments);
    const Vector3d _x_root;
    static SparseMatrixXd GenerateBase(int num_segments);
    static VectorXd GenerateShift(int num_segments, const Vector3d& first_control_point);
	MatrixXd GetChildProjection(double distance) const;
};

