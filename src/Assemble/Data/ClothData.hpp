#pragma once
#include "SampledData.hpp"
#include "JsonUtil.h"

struct ClothData : public SampledObjectData {
	explicit ClothData(const json& config);
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
    ClothData(
		double rho, double thickness,
		double k_stretch, double k_shear,
		double k_bend_max, double k_bend_min,
		const Vector2d& max_dir,
        const Vector3d& start, const Vector3d& u_end, const Vector3d& v_end,
        int num_u_segments, int num_v_segments,
        double stretch_u = 1, double stretch_v = 1
	);

    ClothData(
		double rho, double thickness,
		double k_stretch, double k_shear,
		double k_bend_max, double k_bend_min,
		const Vector2d& max_bend_dir,
        const VectorXd &x, const MatrixXd &uv_corrd,
        const MatrixXi &topo,
		double stretch_u = 1, double stretch_v = 1
	);
	
    const int _curve_num_points;
    const int _num_triangles;
    int _num_internal_edges;
    const double _k_stretch;
    const double _k_shear;
    const double _stretch_u, _stretch_v;
    MatrixXd _uv_coord;
    VectorXd _area;             // area of each triangle
    VectorX<Matrix2d> _inv;     // inverse of (Delta u, Delta v) for every triangle
    VectorX<Matrix6d> _pFpx;    // derivative of F(deform gradient) against xj, xk for every triangle

    MatrixXi _internal_edge;    // edges that lie inside the cloth
                                // _internal_edge[i] stores the indices of the two nodes
                                // of this edge together with their two connecting nodes
    VectorXd _internal_edge_k_bend;
    VectorXd _internal_edge_length;

	static VectorXd GeneratePosition(const Eigen::Vector3d &start, const Eigen::Vector3d &u_end, const Eigen::Vector3d &v_end, int num_u_segments, int num_v_segments);
	static MatrixXd GenerateUVCoord(const Eigen::Vector3d &start, const Eigen::Vector3d &u_end, const Eigen::Vector3d &v_end, int num_u_segments, int num_v_segments);
	static MatrixXi GenerateTopo(int num_u_segments, int num_v_segments);
	static VectorXd GenerateMass(double rho, double thickness, const VectorXd &uv_coord, const MatrixXi &topo);

};

struct BezierClothData : public ReducedObjectData<ClothData> {
	explicit BezierClothData(const json& config);

    /**
     * @warning TODO: currently, control points should fall into the same plane
     */
    BezierClothData(const VectorXd &control_points, double rho, double thickness, double k_stretch, double k_shear, double k_bend_max, double k_bend_min, const Vector3d& max_dir, int num_u_segments, int num_v_segments, double stretch_u, double stretch_v);

};