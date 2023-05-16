//
// Created by hansljy on 11/15/22.
//

#pragma once

#include "Object/Cloth.hpp"
#include "ReducedObject.hpp"
#include "Render/RenderShape.hpp"
#include "Render/LeafShape.hpp"
#include "Collision/CollisionShape/CollisionShape.h"

class Leaf : public Cloth, public LeafRenderShape, public NullCollisionShape {
public:
    using Cloth::Cloth;
    void Initialize() override {
        Cloth::Initialize();
        LeafRenderShape::PreCompute(this);
        NullCollisionShape::Precompute(this);
    }
    PROXY_COLLISION_SHAPE(NullCollisionShape);
    PROXY_RENDER_SHAPE(LeafRenderShape)
};

class ReducedLeaf : public ReducedObject, public ProxyRenderShape, public NullCollisionShape {
public:
    explicit ReducedLeaf(const json& config);
    ReducedLeaf(double density, double thickness, double k_stretch, double k_shear, double k_bend_max, double k_bend_min,
                int num_u_segments, int num_v_segments, const VectorXd& control_points);

    void Initialize() override {
        ReducedObject::Initialize();
        NullCollisionShape::Precompute(this);
    }

    PROXY_RENDER_SHAPE(ProxyRenderShape)
    PROXY_COLLISION_SHAPE(NullCollisionShape)

protected:
    static VectorXd GenerateX(const VectorXd& control_points);
    static SparseMatrixXd GenerateBase(int num_u_segments, int num_v_segments);
    static VectorXd GenerateShift(const VectorXd & control_points, int num_u_segments, int num_v_segments);
    static Vector2d GenerateMaxBendDir(const VectorXd& control_points);
};
