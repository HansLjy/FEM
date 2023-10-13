#pragma once

#include "SampledData.hpp"

struct PDClothData : public SampledObjectData {
    PDClothData(const PDClothData&) = delete;
    PDClothData(PDClothData&&) = default;

    static PDClothData CreateFromFile(const json& config);

    SparseMatrixXd _quadratic_bending;
    std::vector<double> _rest_length;
    double _spring_stiffness;
    double _bending_stiffness;

protected:
    PDClothData(
        const VectorXd& x,
		const VectorXd& mass,
		const MatrixXi& tet_topo,
		const MatrixXi& face_topo,
		const MatrixXi& edge_topo,
		const std::vector<double>& rest_length,
        double spring_stiffness, double bending_stiffness
    );
};