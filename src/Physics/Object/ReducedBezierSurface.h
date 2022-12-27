//
// Created by hansljy on 11/1/22.
//

#ifndef FEM_REDUCEDBEZIERSURFACE_H
#define FEM_REDUCEDBEZIERSURFACE_H

#include "Object.h"

class ReducedLeaf;

class ReducedBezierSurface : public ReducedObject {
public:
    explicit ReducedBezierSurface(const json& config);

    /**
     * @warning TODO: currently, control points should fall into the same plane
     */
    ReducedBezierSurface(bool collision_enabled, const VectorXd &control_points, double rho, double thickness, double k_stretch, double k_shear,
                         double k_bend_max, double k_bend_min, const Vector3d& max_dir,
                         int num_u_segments, int num_v_segments, double stretch_u = 1, double stretch_v = 1);

    friend class ReducedLeaf;

protected:
    static SparseMatrixXd GetBase(int num_u_segments, int num_v_segments);
    static VectorXd GetShift(int num_u_segments, int num_v_segments);
    static VectorXd GenerateX(const VectorXd& control_points, int num_u_segments, int num_v_segments);
    static MatrixXi GenerateTopo(int num_u_segments, int num_v_segments);
    static VectorXd GenerateUVCoord(const VectorXd& control_points, int num_u_segments, int num_v_segments);
    static Vector2d GenerateDir(const VectorXd& control_points, const Vector3d& max_dir);
};

#endif //FEM_REDUCEDBEZIERSURFACE_H
