//
// Created by hansljy on 10/27/22.
//

#pragma once

#include "ExternalForce/ExternalForceContainer.hpp"
#include "SampledObject.hpp"
#include "Render/RenderShape.hpp"
#include "Collision/CollisionShape/CollisionShape.h"

class Cloth : public SampledObject, public SampledRenderShape, public SampledCollisionShape, public ExternalForceContainer<Cloth> {
public:
    explicit Cloth(const json& config);
    void Initialize() override {
        SampledCollisionShape::Precompute(this);
    }

    /**
     * @param start the lower left point of the cloth
     * @param u_end the lower right point of the cloth
     * @param v_end the upper left point of the cloth
     * This will generate a cloth that lies on the plane
     * spanned by start, u_end, v_end. It is supposed to
     * be in the rest shape, thus we can generate the UV
     * coordinates according to this.
     * Note that v_end - start need not be perpendicular
     * to u_end - start, in this case, the latter is
     * considered as the u direction, while v_end - start
     * minus its u-direction part is considered as v-dir
     */
    Cloth(double rho, double thickness, double k_stretch, double k_shear, double k_bend_max, double k_bend_min, const Vector2d& max_dir,
          const Vector3d& start, const Vector3d& u_end, const Vector3d& v_end,
          int num_u_segments, int num_v_segments,
          double stretch_u = 1, double stretch_v = 1);
    Cloth(double rho, double thickness, double k_stretch, double k_shear, double k_bend_max, double k_bend_min, const Vector2d& max_bend_dir,
          const VectorXd &x, const VectorXd &uv_corrd,
          const MatrixXi &topo, double stretch_u = 1, double stretch_v = 1);

	double GetPotential(const Ref<const VectorXd> &x) const override;
	VectorXd GetPotentialGradient(const Ref<const VectorXd> &x) const override;
	void GetPotentialHessian(const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const override;

    PROXY_EXTERNAL_FORCES_TO_CONTAINER(Cloth)
    PROXY_RENDER_SHAPE(SampledRenderShape)
    PROXY_COLLISION_SHAPE(SampledCollisionShape)

    friend SampledObjectGravity<Cloth>;

protected:
    const int _curve_num_points;
    const int _num_triangles;
    int _num_internal_edges;
    const double _k_stretch;
    const double _k_shear;
    const double _stretch_u, _stretch_v;
    VectorXd _uv_coord;
    VectorXd _area;             // area of each triangle
    VectorX<Matrix2d> _inv;     // inverse of (Delta u, Delta v) for every triangle
    VectorX<Matrix6d> _pFpx;    // derivative of F(deform gradient) against xj, xk for every triangle

    MatrixXi _internal_edge;    // edges that lie inside the cloth
                                // _internal_edge[i] stores the indices of the two nodes
                                // of this edge together with their two connecting nodes
    VectorXd _internal_edge_k_bend;
    VectorXd _internal_edge_length;

public:
    static VectorXd GeneratePosition(const Vector3d& start, const Vector3d& u_end, const Vector3d& v_end,
                                     int num_u_segments, int num_v_segments);
    static VectorXd GenerateUVCoord(const Vector3d& start, const Vector3d& u_end, const Vector3d& v_end,
                                    int num_u_segments, int num_v_segments);
    static MatrixXi GenerateTopo(int num_u_segments, int num_v_segments);
    static VectorXd GenerateMass(double rho, double thickness, const VectorXd &uv_coord, const MatrixXi &topo);
	static Eigen::Matrix<double, 9, 5> CalculatePFPX(const Vector3d& e0, const Vector3d& e1, const Vector3d& e2);
};

#include "ReducedObject.hpp"

class ReducedCloth : public ReducedObject, public ProxyRenderShape, public NullCollisionShape {
public:
    explicit ReducedCloth(const json& config);
    void Initialize() override {
        NullCollisionShape::Precompute(this);
    }

    /**
     * @warning TODO: currently, control points should fall into the same plane
     */
    ReducedCloth(const VectorXd &control_points, double rho, double thickness, double k_stretch, double k_shear,
                 double k_bend_max, double k_bend_min, const Vector3d& max_dir,
                 int num_u_segments, int num_v_segments, double stretch_u, double stretch_v);

    PROXY_RENDER_SHAPE(ProxyRenderShape)
    PROXY_COLLISION_SHAPE(NullCollisionShape)
};