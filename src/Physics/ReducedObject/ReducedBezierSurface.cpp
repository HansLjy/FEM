//
// Created by hansljy on 11/1/22.
//

#include "ReducedBezierSurface.h"
#include "Cloth/Cloth.h"

ReducedBezierSurface::ReducedBezierSurface(const VectorXd &control_points, double rho, double k_stretch, double k_shear,
                                           double k_bend, int num_u_segments, int num_v_segments, double stretch_u, double stretch_v)
    : ReducedObject(control_points,
                    Cloth(rho, k_stretch, k_shear, k_bend,
                          GenerateX(control_points, num_u_segments, num_v_segments),
                          GenerateUVCoord(control_points, num_u_segments, num_v_segments),
                          GenerateTopo(num_u_segments, num_v_segments),
                          stretch_u,
                          stretch_v
                    ),
                    GetBase(num_u_segments, num_v_segments)) {}



SparseMatrixXd ReducedBezierSurface::GetBase(int num_u_segments, int num_v_segments) {
    for ()
}

VectorXd ReducedBezierSurface::GenerateX(const Eigen::VectorXd &control_points, int num_u_segments, int num_v_segments) {

}

MatrixXi ReducedBezierSurface::GenerateTopo(int num_u_segments, int num_v_segments) {

}

VectorXd ReducedBezierSurface::GenerateUVCoord(const Eigen::VectorXd &control_points, int num_u_segments, int num_v_segments) {

}