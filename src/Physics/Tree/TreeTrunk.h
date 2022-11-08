//
// Created by hansljy on 11/7/22.
//

#ifndef FEM_TREETRUNK_H
#define FEM_TREETRUNK_H

#include "ReducedObject/ReducedBezierCurve.h"

class TreeTrunk : public ReducedBezierCurve {
public:

    TreeTrunk(int num_segments, double rho, double alpha_max, double alpha_min,
              const VectorXd &control_points, const Vector3d& x_root);

    DERIVED_DECLARE_CLONE(Object)

    void SetFrameInfo(const Vector3d& x_frame, const Vector3d& v_frame, const Vector3d& a_frame,
                      const Vector3d& angular_velocity, const Vector3d& angular_acceleration,
                      const Matrix3d& frame);

    /**
     * @param a the acceleration of control points (general coordinates)
     * @note all const reference are input, all non-const reference are output
     * @warning
     *  1. call this function after the position and velocity has been updated
     *  2. position should be monotonic increasing
     *  3. for efficiency, this function does not clear those output vectors
     *     before it dumps into them
     */
    void CalculateChildrenFrame(const VectorXd &a, const std::vector<double> &position,
                                const std::vector<Matrix3d> &rest_rotation_child,
                                std::vector<Vector3d> &x_child, std::vector<Vector3d> &v_child,
                                std::vector<Vector3d> &a_child,
                                std::vector<Vector3d> &omega_child, std::vector<Vector3d> &alpha_child,
                                std::vector<Matrix3d> &rotation_child);

protected:


    Vector3d _frame_x;
    Vector3d _frame_v;
    Vector3d _frame_a;
    Vector3d _frame_angular_velocity;
    Vector3d _frame_angular_acceleration;
    Matrix3d _frame_rotation;

    const Vector3d _x_root; // every tree trunk has a root, which is supposed
                            // to be in fixed position in the material frame
                            // the root will move along with the parent component
};

#endif //FEM_TREETRUNK_H
