#pragma once

#include "ClothData.hpp"
#include "FileIO.hpp"

struct ReconstructedMeshData : public ClothData {
	ReconstructedMeshData(
		double density,
		double k_stretch, double k_shear, double k_bend,
		const VectorXd &x,
        const MatrixXi &topo
	);
};