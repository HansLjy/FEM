#include "PlanarEmbedding.hpp"
#include <igl/triangle/scaf.h>
#include <igl/arap.h>
#include <igl/boundary_loop.h>
#include <igl/harmonic.h>
#include <igl/map_vertices_to_circle.h>
#include <igl/topological_hole_fill.h>

// From https://github.com/libigl/libigl/blob/main/tutorial/710_SCAF/main.cpp
MatrixXd GetPlaneEmbedding(const MatrixXd& V, const MatrixXi& F) {
	MatrixXd bnd_uv, uv_init;

	VectorXd M;
	igl::doublearea(V, F, M);
	std::vector<std::vector<int>> all_bnds;
	igl::boundary_loop(F, all_bnds);

	// Heuristic primary boundary choice: longest
	auto primary_bnd = std::max_element(all_bnds.begin(), all_bnds.end(), [](const std::vector<int> &a, const std::vector<int> &b) { return a.size()<b.size(); });

	VectorXi bnd = Eigen::Map<VectorXi>(primary_bnd->data(), primary_bnd->size());

	igl::map_vertices_to_circle(V, bnd, bnd_uv);
	bnd_uv *= sqrt(M.sum() / (2 * igl::PI));
	if (all_bnds.size() == 1)
	{
	if (bnd.rows() == V.rows()) // case: all vertex on boundary
	{
		uv_init.resize(V.rows(), 2);
		for (int i = 0; i < bnd.rows(); i++)
		uv_init.row(bnd(i)) = bnd_uv.row(i);
	}
	else
	{
		igl::harmonic(V, F, bnd, bnd_uv, 1, uv_init);
		if (igl::flipped_triangles(uv_init, F).size() != 0)
		igl::harmonic(F, bnd, bnd_uv, 1, uv_init); // fallback uniform laplacian
	}
	}
	else
	{
		// if there is a hole, fill it and erase additional vertices.
		all_bnds.erase(primary_bnd);
		MatrixXi F_filled;
		igl::topological_hole_fill(F, all_bnds, F_filled);
		igl::harmonic(F_filled, bnd, bnd_uv ,1, uv_init);
		uv_init.conservativeResize(V.rows(), 2);
	}

	VectorXi b; MatrixXd bc;

	igl::triangle::SCAFData scaf_data;
	igl::triangle::scaf_precompute(V, F, uv_init, scaf_data, igl::MappingEnergyType::SYMMETRIC_DIRICHLET, b, bc, 0);

	return scaf_data.w_uv.topRows(V.rows());
}