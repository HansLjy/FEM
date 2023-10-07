#pragma once

#include "ClothData.hpp"
#include "FileIO.hpp"

struct ReconstructedMeshData : public FileIOHelper, public ClothData {
	ReconstructedMeshData(const json& config);

	ReconstructedMeshData(
		double rho,
		double k_stretch, double k_shear, double k_bend,
		const VectorXd &x,
        const MatrixXi &topo
	);
};