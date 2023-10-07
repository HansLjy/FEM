#include "ReconstructedMeshData.hpp"
#include "Embedding/PlanarEmbedding.hpp"

ReconstructedMeshData::ReconstructedMeshData(const json& config)
	: ReconstructedMeshData(
		config["density"],
		config["k-stretch"], config["k-shear"], config["k-bend"],
		(FileIOHelper::ReadMesh(config["filename"], config["centered"]), _the_vertices),
		(FileIOHelper::ReadMesh(config["filename"], config["centered"]), _the_topo)
	  ) {}

ReconstructedMeshData::ReconstructedMeshData(
	double rho,
	double k_stretch, double k_shear, double k_bend,
	const VectorXd &x,
	const MatrixXi &topo
) : ClothData(rho, 1, k_stretch, k_shear, k_bend, k_bend, (Vector2d() << 1, 0).finished(), x, GetPlaneEmbedding(x, topo), topo) {
	for (int i = 0; i < _num_points; i++) {
		_x.segment<3>(i * 3) << _uv_coord(i, 0), _uv_coord(i, 1), 0;
	}
}