#include "ReconstructedMeshData.hpp"
#include "Embedding/PlanarEmbedding.hpp"
#include "InitUtils.hpp"

ReconstructedMeshData::ReconstructedMeshData(
	double density,
	double k_stretch, double k_shear, double k_bend,
	const VectorXd &x,
	const MatrixXi &topo) :
	ClothData(
		x, topo,
		InitializationUtils::GenerateMass2D(x, density, topo),
		density,
		k_stretch, k_shear,
		k_bend, k_bend,
		(Vector2d() << 1, 0).finished(),
		GetPlaneEmbedding(StackVector<double, 3>(x), topo),
		1, 1
	) {
	for (int i = 0; i < _num_points; i++) {
		_x.segment<3>(i * 3) << _uv_coord(i, 0), _uv_coord(i, 1), 0;
	}
}