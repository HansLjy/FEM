#include "CurveEnergyModel.hpp"

double CurveEnergyFunction::GetPotential(
	int curve_num_points,
	double stiffness, const VectorXd& alpha,
	const VectorXd& rest_length, const VectorXd& voronoi_length,
	const Ref<const VectorXd> &x
) {

	double potential = 0;
    Vector3d x_current = x.segment<3>(3);
    Vector3d e_prev = x_current - x.segment<3>(0);

    potential += 0.5 * stiffness * (e_prev.norm() - rest_length(0)) * (e_prev.norm() - rest_length(0));

    // (i - 1) -- e_prev --> (i, x_current) -- e_current --> (i + 1, x_next)
    for (int i = 1; i < curve_num_points - 1; i++) {
        Vector3d x_next = x.segment<3>(3 * (i + 1));
        Vector3d e_current = x_next - x_current;

        potential += 0.5 * stiffness * (e_current.norm() - rest_length(i)) * (e_current.norm() - rest_length(i));

        Vector3d kB = 2 * e_prev.cross(e_current) / (e_current.norm() * e_prev.norm() + e_prev.dot(e_current));
        potential += kB.dot(kB) / voronoi_length(i) * alpha(i);

        e_prev = e_current;
        x_current = x_next;
    }
    return potential;
}

VectorXd CurveEnergyFunction::GetPotentialGradient(
	int curve_num_points,
	double stiffness, const VectorXd& alpha,
	const VectorXd& rest_length, const VectorXd& voronoi_length,
	const Ref<const VectorXd> &x
) {
	VectorXd gradient;
    gradient.resizeLike(x);
    gradient.setZero();

    Vector3d x_current = x.block<3, 1>(3, 0);
    Vector3d e_prev = x_current - x.block<3, 1>(0, 0);

    // (i - 1) -- e_prev --> (i, x_current) -- e_current --> (i + 1, x_next)
    for (int i = 1; i < curve_num_points - 1; i++) {
        Vector3d x_next = x.block<3, 1>(3 * (i + 1), 0);
        Vector3d e_current = x_next - x_current;

        const double cur_norm = e_current.norm();
        const double prev_norm = e_prev.norm();
        const double cur_div_prev = cur_norm / prev_norm;
        const double prev_div_cur = prev_norm / cur_norm;
        const double denominator = prev_norm * cur_norm + e_prev.dot(e_current);
        Vector3d kB = 2 * e_prev.cross(e_current) / denominator;
        Matrix3d nabla_prev = (2 * HatMatrix(e_current) + kB * e_current.transpose() + cur_div_prev * kB * e_prev.transpose()) / denominator;
        Matrix3d nabla_next = (2 * HatMatrix(e_prev) - kB * e_prev.transpose() - prev_div_cur * kB * e_current.transpose()) / denominator;
        Matrix3d nabla_current = - nabla_prev - nabla_next;

        const double coefficient = 2 * alpha(i) / voronoi_length(i);
        Vector3d contribute_prev = coefficient * kB.transpose() * nabla_prev;
        Vector3d contribute_current = coefficient * kB.transpose() * nabla_current;
        Vector3d contribute_next = coefficient * kB.transpose() * nabla_next;

        gradient.block<3, 1>(3 * (i - 1), 0) += contribute_prev;
        gradient.block<3, 1>(3 * i, 0) += contribute_current;
        gradient.block<3, 1>(3 * (i + 1), 0) += contribute_next;

        e_prev = e_current;
        x_current = x_next;
	}

    for (int i = 0, j = 0; i < curve_num_points - 1; i++, j += 3) {
        Vector3d e = x.segment<3>(j + 3) - x.segment<3>(j);
        Vector3d contribution = stiffness * (e.norm() - rest_length(i)) * e.normalized();
        gradient.segment<3>(j) -= contribution;
        gradient.segment<3>(j + 3) += contribution;
    }

    return gradient;
}

void CurveEnergyFunction::GetPotentialHessian(
	int curve_num_points,
	double stiffness, const VectorXd& alpha,
	const VectorXd& rest_length, const VectorXd& voronoi_length,
	const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset
) {
	Vector3d x_current = x.segment<3>(3);
    Vector3d e_prev = x_current - x.segment<3>(0);

    // (i - 1) -- e_prev --> (i, x_current) -- e_current --> (i + 1, x_next)
    for (int i = 1, ii = 0; i < curve_num_points - 1; i++, ii += 3) {
        Vector3d x_next = x.block<3, 1>(3 * (i + 1), 0);
        Vector3d e_current = x_next - x_current;

        const double prev_norm = e_prev.norm();
        const double cur_norm = e_current.norm();
        const double prev_div_cur = prev_norm / cur_norm;
        const double cur_div_prev = cur_norm / prev_norm;
        const double denominator = prev_norm * cur_norm + e_prev.dot(e_current);
        Vector3d kB = 2 * e_prev.cross(e_current) / denominator;

        const Matrix3d hat_e_current = HatMatrix(e_current);
        const Matrix3d hat_e_prev = HatMatrix(e_prev);

        Eigen::Vector<Matrix3d, 3> p_kB;

        p_kB(0) = (- 2 * hat_e_current + e_current * kB.transpose() + cur_div_prev * e_prev * kB.transpose()) / denominator;
        p_kB(2) = (- 2 * hat_e_prev - e_prev * kB.transpose() - prev_div_cur * e_current * kB.transpose()) / denominator;
        p_kB(1) = - p_kB(2) - p_kB(0);

        Matrix9d hessian;
        hessian.setZero();

        static const Matrix3d I3 = Matrix3d::Identity();
        const auto& kB_kron_I3 = Eigen::KroneckerProduct<Vector3d, Matrix3d>(kB, I3);
        const auto& I3_kron_e_current = Eigen::KroneckerProduct<Matrix3d, Vector3d>(I3, e_current);
        const auto& I3_kron_e_prev = Eigen::KroneckerProduct<Matrix3d, Vector3d>(I3, e_prev);
        const auto& kB_kron_e_current = Eigen::KroneckerProduct<Vector3d, Vector3d>(kB, e_current);
        const auto& kB_kron_e_prev = Eigen::KroneckerProduct<Vector3d, Vector3d>(kB, e_prev);

        static const Matrix<double, 9, 3> vec_hat_matrix(GetVecHatMatrix());

        Matrix<Matrix<double, 3, 9>, 3, 3> p2_kB;

        Vector3d d_cur_div_prev = e_current + cur_div_prev * e_prev;
        Vector3d d_prev_div_cur = e_prev + prev_div_cur * e_current;

        Matrix<double, 9, 3> I3_prev_div_cur = (I3_kron_e_prev + prev_div_cur * I3_kron_e_current) / (denominator * denominator);
        Matrix<double, 9, 3> I3_cur_div_prev = (I3_kron_e_current + cur_div_prev * I3_kron_e_prev) / (denominator * denominator);
        Vector9d kB_prev_div_cur = kB_kron_e_prev + prev_div_cur * kB_kron_e_current;
        Vector9d kB_cur_div_prev = kB_kron_e_current + cur_div_prev * kB_kron_e_prev;
        Vector9d d_denominator_cur = (-2 * vec_hat_matrix * e_current + kB_cur_div_prev) / (denominator * denominator);
        Vector9d d_denominator_prev = (-2 * vec_hat_matrix * e_prev - kB_prev_div_cur) / (denominator * denominator);
        Matrix3d nominator_prev = 2 * hat_e_prev - kB * d_prev_div_cur.transpose();
        Matrix3d nominator_cur = -2 * hat_e_current - kB * d_cur_div_prev.transpose();

        p2_kB(0, 0) = -(
            I3_cur_div_prev * nominator_cur
            + cur_div_prev * kB_kron_I3 / denominator
            - kB_kron_e_prev * e_prev.transpose() * cur_div_prev / denominator / (prev_norm * prev_norm)
            - d_denominator_cur * d_cur_div_prev.transpose()
        ).transpose();

        p2_kB(2, 2) = (
            - I3_prev_div_cur * nominator_prev
            - prev_div_cur * kB_kron_I3 / denominator
            + kB_kron_e_current * e_current.transpose() * prev_div_cur / denominator / (cur_norm * cur_norm)
            - d_denominator_prev * d_prev_div_cur.transpose()
        ).transpose();

        p2_kB(0, 2) = (
            - 2 * vec_hat_matrix / denominator
            + kB_kron_I3 / denominator
            + I3_cur_div_prev * nominator_prev
            + kB_kron_e_prev * e_current.transpose() / (denominator * cur_norm * prev_norm)
            - d_denominator_cur * d_prev_div_cur.transpose()
        ).transpose();

        p2_kB(2, 0) = -(
            - 2 * vec_hat_matrix / denominator
            - kB_kron_I3 / denominator
            - I3_prev_div_cur * nominator_cur
            - kB_kron_e_current * e_prev.transpose() / (denominator * cur_norm * prev_norm)
            - d_denominator_prev * d_cur_div_prev.transpose()
        ).transpose();

        p2_kB(1, 0) = - p2_kB(0, 0) - p2_kB(2, 0);
        p2_kB(1, 2) = - p2_kB(0, 2) - p2_kB(2, 2);
        p2_kB(0, 1) = - p2_kB(0, 0) - p2_kB(0, 2);
        p2_kB(2, 1) = - p2_kB(2, 2) - p2_kB(2, 0);
        p2_kB(1, 1) = p2_kB(0, 0) + p2_kB(2, 0) + p2_kB(0, 2) + p2_kB(2, 2);

        const double coeff = 2 * alpha(i) / voronoi_length(i);
        for (int j = 0, jj = 0; j < 3; j++, jj += 3) {
            for (int k = 0, kk = 0; k < 3; k++, kk += 3) {
                hessian.block<3, 3>(jj, kk) = coeff * (
                        p2_kB(j, k) * kB_kron_I3
                        + p_kB(k) * p_kB(j).transpose()
                ).transpose();
            }
        }

        for (int j = 0; j < 9; j++) {
            for (int k = 0; k < 9; k++) {
                if(hessian(j, k) != 0) {
                    coo.push_back(Tripletd(ii + j + x_offset, ii + k + y_offset, hessian(j, k)));
                }
            }
        }

        e_prev = e_current;
        x_current = x_next;
    }

    for (int i = 0, ii = 0; i < curve_num_points - 1; i++, ii += 3) {
        Vector3d e = x.segment<3>(ii + 3) - x.segment<3>(ii);
        const double e_norm = e.norm();
        const Matrix3d I3 = Matrix3d::Identity();
        Matrix6d hessian;
        hessian.setZero();

        Matrix3d sub_hessian = stiffness * (1 - rest_length(i) / e.norm()) * I3
                             + stiffness * rest_length(i) / (e_norm * e_norm * e_norm) * e * e.transpose();
        hessian.block<3, 3>(0, 0) = hessian.block<3, 3>(3, 3) = sub_hessian;
        hessian.block<3, 3>(0, 3) = hessian.block<3, 3>(3, 0) = - sub_hessian;

        for (int j = 0; j < 6; j++) {
            for (int k = 0; k < 6; k++) {
                if (hessian(j, k) != 0) {
                    coo.push_back(Tripletd(x_offset + ii + j, y_offset + ii + k, hessian(j, k)));
                }
            }
        }
    }

}