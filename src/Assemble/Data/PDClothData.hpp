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
        const VectorXd& x, const MatrixXi& topo, const MatrixXd& mass,
        double spring_stiffness, double bending_stiffness
    );
};