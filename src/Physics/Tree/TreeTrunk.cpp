//
// Created by hansljy on 11/7/22.
//

#include "TreeTrunk.h"

DEFINE_CLONE(Object, TreeTrunk)

TreeTrunk::TreeTrunk(int num_segments, double rho, double alpha_max, double alpha_min,
                     const Eigen::VectorXd &control_points, const Eigen::Vector3d &x_root)
                     : ReducedBezierCurve(num_segments, rho, alpha_max, alpha_min, control_points), _x_root(x_root) {}

void
TreeTrunk::SetFrameInfo(const Eigen::Vector3d &x_frame, const Eigen::Vector3d &v_frame, const Eigen::Vector3d &a_frame,
                        const Eigen::Vector3d &angular_velocity, const Eigen::Vector3d &angular_acceleration,
                        const Eigen::Matrix3d &frame) {
    _frame_x = x_frame;
    _frame_v = v_frame;
    _frame_a = a_frame;
    _frame_angular_velocity = angular_velocity;
    _frame_angular_acceleration = angular_acceleration;
    _frame_rotation = frame;
}

void TreeTrunk::CalculateChildrenFrame(const VectorXd &a, const std::vector<double> &position,
                                       const std::vector<Matrix3d> &rest_rotation_child,
                                       std::vector<Vector3d> &x_child, std::vector<Vector3d> &v_child,
                                       std::vector<Vector3d> &a_child,
                                       std::vector<Vector3d> &omega_child, std::vector<Vector3d> &alpha_child,
                                       std::vector<Matrix3d> &rotation_child) {
    const auto& points = _proxy->GetCoordinate();
    const auto& velocity = _proxy->GetVelocity();
    const VectorXd acceleration = _base * a;
    const int num_points = points.size() / 3;
    const int num_segments = num_points - 1;
    const double delta_t = 1.0 / num_segments;
    const int num_children = position.size();

    double current_t = 0;
    int num_children_processed = 0;
    Matrix3d rotation_accumulated = Matrix3d::Identity();    // the root frame is supposed to be I3
    Vector3d x_prev = _x_root;                          // x_{i - 1}
    Vector3d x_current = points.segment<3>(0);          // x_i
    Vector3d v_prev = Vector3d::Zero();                 // v_{i - 1}
    Vector3d v_current = velocity.segment<3>(0);        // v_i
    Vector3d a_prev = Vector3d::Zero();                 // a_{i - 1}
    Vector3d a_current = acceleration.segment<3>(0);    // a_i

    Matrix3d omega_accumulated = Matrix3d::Zero();
    Matrix3d alpha_accumulated = Matrix3d::Zero();

    for (int i = 0, j = 0; i < num_segments; i++, j += 3, current_t += delta_t) {
        Vector3d x_next = points.segment<3>(j + 3);         // x_{i + 1}
        Vector3d v_next = velocity.segment<3>(j + 3);       // v_{i + 1}
        Vector3d a_next = acceleration.segment<3>(j + 3);   // a_{i + 1}

        const Vector3d e_prev = x_current - x_prev;               // e^{i - 1}
        const Vector3d e_current = x_next - x_current;            // e^i
        const double e_prev_norm = e_prev.norm();
        const double e_current_norm = e_current.norm();

        const Vector3d de_prev_dt = v_current - v_prev;
        const Vector3d de_current_dt = v_next - v_current;
        const Vector3d d2e_prev_dt2 = a_current - a_prev;
        const Vector3d d2e_current_dt2 = a_next - a_current;

        const Vector3d t_prev = e_prev.normalized();
        const Vector3d t_current = e_current.normalized();

        const double e_prev_norm_3 = e_prev_norm * e_prev_norm * e_prev_norm;
        const double e_prev_norm_5 = e_prev_norm_3 * e_prev_norm * e_prev_norm;

        const Vector3d dt_prev_dt = de_prev_dt / e_prev_norm
                                  - e_prev * (e_prev.dot(de_prev_dt)) / e_prev_norm_3;
        const Vector3d d2t_prev_dt2 = d2e_prev_dt2 / e_prev_norm
                                    - 2 * de_prev_dt * (e_prev.dot(de_prev_dt)) / e_prev_norm_3
                                    - e_prev * (de_prev_dt.dot(de_prev_dt) + e_prev.dot(d2e_prev_dt2)) / e_prev_norm_3
                                    + 3 * e_prev * (e_prev.dot(de_prev_dt) * e_prev.dot(de_prev_dt)) / e_prev_norm_5;

        const double e_current_norm_3 = e_current_norm * e_current_norm * e_current_norm;
        const double e_current_norm_5 = e_current_norm_3 * e_current_norm * e_current_norm;

        const Vector3d dt_current_dt = de_current_dt / e_current_norm
                                    - e_current * (e_current.dot(de_current_dt)) / e_current_norm_3;
        const Vector3d d2t_current_dt2 = d2e_current_dt2 / e_current_norm
                                      - 2 * de_current_dt * (e_current.dot(de_current_dt)) / e_current_norm_3
                                      - e_current * (de_current_dt.dot(de_current_dt) + e_current.dot(d2e_current_dt2)) / e_current_norm_3
                                      + 3 * e_current * (e_current.dot(de_current_dt) * e_current.dot(de_current_dt)) / e_current_norm_5;


        const Vector3d Si = t_prev.cross(t_current);
        const double Ci = t_prev.dot(t_current);

        const Vector3d dSi_dt = dt_prev_dt.cross(t_current) + t_prev.cross(dt_current_dt);
        const Vector3d d2Si_dt2 = d2t_prev_dt2.cross(t_current) + 2 * dt_prev_dt.cross(dt_current_dt) + t_prev.cross(d2t_current_dt2);

        const double dCi_dt = dt_prev_dt.dot(t_current) + t_prev.dot(dt_current_dt);
        const double d2Ci_dt2 = d2t_prev_dt2.dot(t_current) + 2 * dt_prev_dt.dot(dt_current_dt) + t_prev.dot(d2t_current_dt2);

        const Matrix3d Ri = Ci * Matrix3d::Identity() + (1.0 / (1.0 + Ci)) * Si * Si.transpose() + HatMatrix(Si);
        const Matrix3d dRi_dt = dCi_dt * Matrix3d::Identity()
                              - dCi_dt / ((1 + Ci) * (1 + Ci)) * Si * Si.transpose()
                              + 1 / (1 + Ci) * (dSi_dt * Si.transpose() + Si * dSi_dt.transpose())
                              + HatMatrix(dSi_dt);
        const Matrix3d d2Ri_dt2 = d2Ci_dt2 * Matrix3d::Identity()
                                + (- d2Ci_dt2 / ((1 + Ci) * (1 + Ci)) + 2 * dCi_dt * dCi_dt / ((1 + Ci) * (1 + Ci) * (1 + Ci))) * Si * Si.transpose()
                                - 2 * dCi_dt / ((1 + Ci) * (1 + Ci)) * (dSi_dt * Si.transpose() + Si * dSi_dt.transpose())
                                + 1.0 / (1 + Ci) * (d2Si_dt2 * Si.transpose() + 2 * dSi_dt * dSi_dt.transpose() + Si * d2Si_dt2.transpose())
                                + HatMatrix(d2Si_dt2);

        assert((Ri * Ri.transpose() - Matrix3d::Identity()).norm() < 1e-5);

        rotation_accumulated = Ri * rotation_accumulated;
        Matrix3d current_omega = dRi_dt * Ri.transpose();
        Matrix3d current_alpha = d2Ri_dt2 * Ri.transpose() - current_omega * current_omega;

        alpha_accumulated = current_alpha
                          + current_omega * Ri * omega_accumulated * Ri.transpose()
                          - Ri * omega_accumulated * Ri.transpose() * current_omega
                          + Ri * alpha_accumulated * Ri.transpose();
        omega_accumulated = current_omega + Ri * omega_accumulated * Ri.transpose();

        while (num_children_processed < num_children && position[num_children_processed] <= current_t + delta_t) {
            // calculate the physics quantities relative to current frame
            rotation_child.push_back(rotation_accumulated);
            const double portion = (position[num_children_processed] - current_t) / delta_t;
                // how far is this point from the beginning of the segments
            x_child.push_back(
                x_current + portion * (x_next - x_current)
            );
            v_child.push_back(
                v_current + portion * (v_next - v_current)
            );
            a_child.push_back(
                a_current + portion * (a_next - a_current)
            );
            omega_child.push_back(SkewVector(omega_accumulated));
            alpha_child.push_back(SkewVector(alpha_accumulated));
            num_children_processed++;
        }

        x_prev = x_current;
        x_current = x_next;
        v_prev = v_current;
        v_current = v_next;
        a_prev = a_current;
        a_current = a_next;
    }

    for (int i = 0; i < num_children; i++) {
        const auto& R = _frame_rotation;
        Vector3d x_rel = x_child[i], v_rel = v_child[i], a_rel = a_child[i];
        x_child[i] = _frame_x + R * x_rel;
        v_child[i] = _frame_v + _frame_angular_velocity.cross(R * x_rel) + R * v_rel;
        a_child[i] = _frame_a + _frame_angular_acceleration.cross(R * x_rel)
                   + _frame_angular_velocity.cross(_frame_angular_velocity.cross(R * x_rel))
                   + 2 * _frame_angular_velocity.cross(R * v_rel) + R * a_rel;
        rotation_child[i] = R * rotation_child[i] * rest_rotation_child[i];
        const auto omega = omega_child[i];
        const auto alpha = alpha_child[i];
        omega_child[i] = _frame_angular_velocity + SkewVector(R * HatMatrix(omega) * R.transpose());
        alpha_child[i] = _frame_angular_acceleration + SkewVector(
            R * HatMatrix(alpha) * R.transpose()
            + HatMatrix(_frame_angular_velocity) * R * HatMatrix(omega) * R.transpose()
            - R * HatMatrix(omega) * R.transpose() * HatMatrix(_frame_angular_velocity)
        );
    }
}