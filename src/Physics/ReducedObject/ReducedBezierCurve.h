//
// Created by hansljy on 10/18/22.
//

#ifndef FEM_REDUCEDBEZIERCURVE_H
#define FEM_REDUCEDBEZIERCURVE_H

#include "ReducedObject.h"
#include "Curve/InextensibleCurve.h"

class ReducedTreeTrunk;

class ReducedBezierCurve : public ReducedObject {
public:
    explicit ReducedBezierCurve(const json& config);
    ReducedBezierCurve(int num_segments, double rho, double alpha_max, double alpha_min,
                       const VectorXd &control_points);

    DERIVED_DECLARE_CLONE(Object)

    friend class ReducedTreeTrunk;
private:
    static VectorXd GenerateSamplePoints(int num_segments, const VectorXd& control_points);
    static SparseMatrixXd GenerateBase(int num_segment);
    static VectorXd GenerateShift(int num_segments);

};

#endif //FEM_REDUCEDBEZIERCURVE_H
