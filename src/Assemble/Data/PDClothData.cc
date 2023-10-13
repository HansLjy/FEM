#include "InitUtils.hpp"
#include "PDClothData.hpp"
#include "FileIO.hpp"
#include "ExternalForce/ExternalForce.hpp"

template<>
Factory<ExternalForce<PDClothData>>* Factory<ExternalForce<PDClothData>>::_the_factory = nullptr;

PDClothData PDClothData::CreateFromFile(const json &config) {
    VectorXd x;
    MatrixXi topo;
    FileIOUtils::ReadMesh(config["filename"], config["centered"], x, topo);
    VectorXd mass = InitializationUtils::GenerateMass2D(x, config["density"], topo);
    return {
        x, topo, mass,
        config["spring-stiffness"],
        config["bending-stiffness"]
    };
}

PDClothData::PDClothData(
    const VectorXd& x, const MatrixXi& topo, const MatrixXd& mass,
    double spring_stiffness, double bending_stiffness
) : SampledObjectData(x, mass, 2, topo) {}
