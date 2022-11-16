//
// Created by hansljy on 11/15/22.
//

#ifndef FEM_REDUCEDLEAF_H
#define FEM_REDUCEDLEAF_H

#include "ReducedObject/ReducedObject.h"
#include "Cloth/Cloth.h"

class ReducedLeaf : public ReducedObject {
public:
    explicit ReducedLeaf(const json& config);
    ReducedLeaf(double density, double thickness, double k_stretch, double k_shear, double k_bend_max, double k_bend_min,
                int u_segments, int v_segments, const VectorXd& control_points);

    DERIVED_DECLARE_CLONE(Object)

protected:
    static VectorXd GenerateX(const VectorXd& control_points);
    static VectorXd GenerateClothX(const VectorXd& control_points, int u_segments, int v_segments);
    static VectorXd GenerateClothUV(const VectorXd& control_points, int u_segments, int v_segments);
    static MatrixXi GenerateClothTopo(int u_segments, int v_segments);
    static SparseMatrixXd GenerateBase(int u_segments, int v_segments);
    static VectorXd GenerateShift(const VectorXd & control_points, int u_segments, int v_segments);
    static Vector2d GenerateMaxBendDir(const VectorXd& control_points);
};

#endif //FEM_REDUCEDLEAF_H
