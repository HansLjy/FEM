//
// Created by hansljy on 11/1/22.
//

#ifndef FEM_REDUCEDBEZIERSURFACE_H
#define FEM_REDUCEDBEZIERSURFACE_H

#include "ReducedObject.h"

class ReducedBezierSurface : public ReducedObject {
public:
    explicit ReducedBezierSurface(const json& config);

    /**
     * @warning TODO: currently, control points should fall into the same plane
     */
    ReducedBezierSurface(const VectorXd &control_points, double rho, double k_stretch, double k_shear, double k_bend,
                         int num_u_segments, int num_v_segments, double stretch_u = 1, double stretch_v = 1);

    DERIVED_DECLARE_CLONE(Object)

protected:
    static SparseMatrixXd GetBase(int num_u_segments, int num_v_segments);
    static VectorXd GenerateX(const VectorXd& control_points, int num_u_segments, int num_v_segments);
    static MatrixXi GenerateTopo(int num_u_segments, int num_v_segments);
    static VectorXd GenerateUVCoord(const VectorXd& control_points, int num_u_segments, int num_v_segments);
};

#endif //FEM_REDUCEDBEZIERSURFACE_H
