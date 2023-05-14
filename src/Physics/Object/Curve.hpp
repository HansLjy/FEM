//
// Created by hansljy on 10/19/22.
//

#ifndef FEM_CURVE_H
#define FEM_CURVE_H

#include "SampledObject.hpp"
#include "Render/CurveShape.hpp"
#include "ExternalForce/ExternalForceContainer.hpp"
#include "Collision/CollisionShape/CollisionShape.h"

class Curve : public SampledObject, public CurveShape, public NullCollisionShape, public ExternalForceContainer<Curve> {
public:
	Curve(const json& config);
    void Initialize() override {
        NullCollisionShape::Precompute(this);
    }

	Curve(double rho, double alpha_max, double alpha_min, const Vector3d &start, const Vector3d &end, int num_segments)
        : Curve(rho, alpha_max, alpha_min, GetX(start, end, num_segments)) {}
    Curve(double rho, double alpha_max, double alpha_min, const VectorXd &x);

	double GetPotential(const Ref<const VectorXd> &x) const override;
	VectorXd GetPotentialGradient(const Ref<const VectorXd> &x) const override;
	void GetPotentialHessian(const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const override;

    PROXY_EXTERNAL_FORCES_TO_CONTAINER(Curve)
    PROXY_RENDER_SHAPE(CurveShape)
    PROXY_COLLISION_SHAPE(NullCollisionShape)
    

    friend SampledObjectGravity<Curve>;

protected:
    const double _k;      		// stiffness of the curve
    int _curve_num_points;      // number of sampled points (end points included)
    VectorXd _alpha;
    VectorXd _x_rest;           // rest shape of the curve
    VectorXd _rest_length;      // length of every edge in the rest shape
    VectorXd _voronoi_length;   // length under the government of one point

    static VectorXd GetX(const Vector3d& start, const Vector3d& end, int num_segments);
    static VectorXd GenerateMass(double rho, const VectorXd &x);
	static MatrixXi GenerateTopo(int n);
};

#include "ReducedObject.hpp"
#include "Collision/CollisionShape/CollisionShape.h"

class ReducedCurve : public ReducedObject, public ProxyRenderShape, public NullCollisionShape {
public:
    explicit ReducedCurve(const json& config);
    ReducedCurve(int num_segments, double rho, double alpha_max, double alpha_min,
                 const VectorXd &control_points);

    PROXY_RENDER_SHAPE(ProxyRenderShape)
    PROXY_COLLISION_SHAPE(NullCollisionShape)
};

#endif //FEM_CURVE_H
