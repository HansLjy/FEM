//
// Created by hansljy on 10/27/22.
//

#ifndef FEM_CLOTH_H
#define FEM_CLOTH_H

#include "Object.h"

class ClothShape;
class ReducedBezierSurface;

class Cloth : public ShapedObject, public SampledObject {
public:
    explicit Cloth(const json& config);

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
    Cloth(double rho, double k_stretch, double k_shear, double k_bend,
          const Vector3d& start, const Vector3d& u_end, const Vector3d& v_end,
          int num_u_segments, int num_v_segments,
          double stretch_u = 1, double stretch_v = 1);
    Cloth(double rho, double k_stretch, double k_shear, double k_bend, const VectorXd &x, const VectorXd &uv_corrd,
          const MatrixXi &topo, double stretch_u = 1, double stretch_v = 1);

    double GetPotential() const override;
    VectorXd GetPotentialGradient() const override;
    void GetPotentialHessian(COO &coo, int x_offset, int y_offset) const override;

    DERIVED_DECLARE_CLONE(Object)

    friend class ClothShape;
    friend class ReducedBezierSurface;

protected:
    const int _num_points;
    const int _num_triangles;
    int _num_internal_edges;
    const double _k_stretch;
    const double _k_shear;
    const double _k_bend;
    const double _stretch_u, _stretch_v;
    VectorXd _uv_coord;
    VectorXd _area;             // area of each triangle
    MatrixXi _topo;             // each row contains the indices of
                                // three nodes that comprise the triangle
    VectorX<Matrix2d> _inv;     // inverse of (Delta u, Delta v) for every triangle
    VectorX<Matrix6d> _pFpx;    // derivative of F(deform gradient) against xj, xk for every triangle

    MatrixXi _internal_edge;    // edges that lie inside the cloth
                                // _internal_edge[i] stores the indices of the two nodes
                                // of this edge together with their two connecting nodes
    VectorXd _internal_edge_length;

protected:
    static Eigen::Matrix<double, 9, 5> CalculatePFPX(const Vector3d& e0, const Vector3d& e1, const Vector3d& e2);
    static VectorXd GeneratePosition(const Vector3d& start, const Vector3d& u_end, const Vector3d& v_end,
                                     int num_u_segments, int num_v_segments);
    static VectorXd GenerateUVCoord(const Vector3d& start, const Vector3d& u_end, const Vector3d& v_end,
                                    int num_u_segments, int num_v_segments);
    static MatrixXi GenerateTopo(int num_u_segments, int num_v_segments);
    static VectorXd GenerateMass(double rho, const VectorXd &uv_coord, const MatrixXi &topo);
};

#endif //FEM_CLOTH_H
