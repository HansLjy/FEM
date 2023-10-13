#pragma once
#include "SampledData.hpp"
#include "JsonUtil.h"

struct ClothData : public SampledObjectData {
    double _k_stretch;
    double _k_shear;
    double _stretch_u, _stretch_v;
    MatrixXd _uv_coord;
    VectorXd _area;             // area of each triangle

    int _num_internal_edges;
    MatrixXi _internal_edge;    // edges that lie inside the cloth
                                // _internal_edge[i] stores the indices of the two nodes
                                // of this edge together with their two connecting nodes
    VectorXd _internal_edge_k_bend;
    VectorXd _internal_edge_length;

    /* For energy calculation */
    std::vector<Matrix2d> _inv;     // inverse of (Delta u, Delta v) for every triangle
    std::vector<Matrix6d> _pFpx;    // derivative of F(deform gradient) against xj, xk for every triangle

protected:
    ClothData(
        const VectorXd& x,
        const MatrixXi& topo,
        const VectorXd& mass,
		double density,
		double k_stretch, double k_shear,
		double k_bend_max, double k_bend_min,
		const Vector2d& max_bend_dir,
        const MatrixXd& uv_coord,
		double stretch_u, double stretch_v
	);
};