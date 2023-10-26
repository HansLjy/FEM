#include "InitUtils.hpp"
#include "PDClothData.hpp"
#include "FileIO.hpp"
#include "ExternalForce/ExternalForce.hpp"
#include "TopoUtil.hpp"

template<>
Factory<ExternalForce<PDClothData>>* Factory<ExternalForce<PDClothData>>::_the_factory = nullptr;

PDClothData PDClothData::CreateFromFile(const json &config) {
    VectorXd x;
    MatrixXi face_topo, tet_topo(0, 4), edge_topo;
	Vector3d rotation_axis = Json2Vec(config["rotation-axis"]);
	double rotation_angle = static_cast<double>(config["rotation-angle"]) / 180 * M_PI;
	Matrix3d rotation = Matrix3d(Eigen::AngleAxis(rotation_angle, rotation_axis));
    FileIOUtils::ReadMesh(
		config["filename"],
		x, face_topo,
		config["centered"],
		rotation,
		Json2Vec(config["translation"])
	);
	TopoUtil::GenerateSurfaceTopo2D(face_topo, edge_topo);
    VectorXd mass = InitializationUtils::GenerateMass2D(x, config["density"], face_topo);
	auto rest_length = InitializationUtils::GetRestLength(x, edge_topo);
    return {
        x, mass, tet_topo, face_topo, edge_topo,
		rest_length,
        config["spring-stiffness"],
        config["bending-stiffness"]
    };
}

PDClothData::PDClothData(
	const VectorXd& x,
	const VectorXd& mass,
	const MatrixXi& tet_topo,
	const MatrixXi& face_topo,
	const MatrixXi& edge_topo,
	const std::vector<double>& rest_length,
	double spring_stiffness,
	double bending_stiffness) :
	SampledObjectData(x, mass, tet_topo, face_topo, edge_topo),
	_rest_length(rest_length),
	_spring_stiffness(spring_stiffness),
	_bending_stiffness(bending_stiffness) {}
