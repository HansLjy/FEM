#include "GridEnergyModel.hpp"

double GridEnergyFunction::GetPotential(
	const Ref<const VectorXd> &x,
	int num_edges, int start_diag_edges, int num_points,
	double stiffness, double diag_stiffness, double ret_stiffness,
	const Ref<const MatrixXi>& edge_topo,
	const Ref<const VectorXd>& rest_length,
	const Ref<const VectorXd>& x_rest
) {
	double energy = 0;
	for (int i = 0; i < start_diag_edges; i++) {
		const auto& indices = edge_topo.row(i);
		const int id1 = indices[0], id2 = indices[1];
		double delta_x = (x.segment<3>(id2 * 3) - x.segment<3>(id1 * 3)).norm() - 
		rest_length(i);
		energy += stiffness * delta_x * delta_x / 2;
	}

	for (int i = start_diag_edges; i < num_edges; i++) {
		const auto& indices = edge_topo.row(i);
		const int id1 = indices[0], id2 = indices[1];
		double delta_x = (x.segment<3>(id2 * 3) - x.segment<3>(id1 * 3)).norm() - 
		rest_length(i);
		energy += diag_stiffness * delta_x * delta_x / 2;
	}

	for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
		double delta_x = (x.segment<3>(i3) - x_rest.segment<3>(i3)).norm();
		energy += ret_stiffness * delta_x * delta_x / 2;
	}
	return energy;

}

VectorXd GridEnergyFunction::GetPotentialGradient(
	const Ref<const VectorXd>& x,
	int num_edges, int start_diag_edges, int num_points,
	double stiffness, double diag_stiffness, double ret_stiffness,
	const Ref<const MatrixXi>& edge_topo,
	const Ref<const VectorXd>& rest_length,
	const Ref<const VectorXd>& x_rest
) {
	VectorXd gradient = VectorXd::Zero(x.size());
	for (int i = 0; i < start_diag_edges; i++) {
		const auto& indices = edge_topo.row(i);
		const int id1 = indices[0], id2 = indices[1];
		Vector3d e = x.segment<3>(3 * id2) - x.segment<3>(3 * id1);
		Vector3d single_gradient = - stiffness * (1 - rest_length(i) / e.norm()) * e;
		gradient.segment<3>(id1 * 3) += single_gradient;
		gradient.segment<3>(id2 * 3) += - single_gradient;
	}

	for (int i = start_diag_edges; i < num_edges; i++) {
		const auto& indices = edge_topo.row(i);
		const int id1 = indices[0], id2 = indices[1];
		Vector3d e = x.segment<3>(3 * id2) - x.segment<3>(3 * id1);
		Vector3d single_gradient = - diag_stiffness * (1 - rest_length(i) / e.norm()) * e;
		gradient.segment<3>(id1 * 3) += single_gradient;
		gradient.segment<3>(id2 * 3) += - single_gradient;
	}

	for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
		Vector3d e = x.segment<3>(i3) - x_rest.segment<3>(i3);
		gradient.segment<3>(i3) += ret_stiffness * e;
	}

	return gradient;
}

void GridEnergyFunction::GetPotentialHessian(
	const Ref<const VectorXd> &x,
	int dof,
	int num_edges, int start_diag_edges, int num_points,
	double stiffness, double diag_stiffness, double ret_stiffness,
	const Ref<const MatrixXi>& edge_topo,
	const Ref<const VectorXd>& rest_length,
	const Ref<const VectorXd>& x_rest,
	COO &coo, int x_offset, int y_offset
) {
	for (int i = 0; i < num_edges; i++) {
		const auto& indices = edge_topo.row(i);
		const int id1 = indices[0], id2 = indices[1];
		Vector3d e = x.segment<3>(3 * id2) - x.segment<3>(3 * id1);
		const double e_norm = e.norm();
		double cur_stiffness = i < start_diag_edges ? stiffness : diag_stiffness;
		Matrix3d single_hession = cur_stiffness * ((1 - rest_length(i) / e_norm) * Matrix3d::Identity()
		                          + rest_length(i) / (e_norm * e_norm * e_norm) * e * e.transpose());
		const int offset1 = id1 * 3, offset2 = id2 * 3;
		
		#ifdef BUILD_TEST
			for (int row = 0; row < 3; row++) {
				for (int col = 0; col < 3; col++) {
					double val = single_hession(row, col);
					coo.push_back(Tripletd(x_offset + offset1 + row, y_offset + offset1 + col, val));
					coo.push_back(Tripletd(x_offset + offset1 + row, y_offset + offset2 + col, -val));
					coo.push_back(Tripletd(x_offset + offset2 + row, y_offset + offset1 + col, -val));
					coo.push_back(Tripletd(x_offset + offset2 + row, y_offset + offset2 + col, val));
				}
			}
		#else
			Matrix6d block_hession;
			block_hession.block<3, 3>(0, 0) = block_hession.block<3, 3>(3, 3) = single_hession;
			block_hession.block<3, 3>(0, 3) = block_hession.block<3, 3>(3, 0) = -single_hession;
			block_hession = PositiveProject(block_hession);
			for (int row = 0; row < 3; row++) {
				for (int col = 0; col < 3; col++) {
					coo.push_back(Tripletd(x_offset + offset1 + row, y_offset + offset1 + col, block_hession(row, col)));
					coo.push_back(Tripletd(x_offset + offset1 + row, y_offset + offset2 + col, block_hession(row, col + 3)));
					coo.push_back(Tripletd(x_offset + offset2 + row, y_offset + offset1 + col, block_hession(row + 3, col)));
					coo.push_back(Tripletd(x_offset + offset2 + row, y_offset + offset2 + col, block_hession(row + 3, col + 3)));
				}
			}
		#endif
	}

	for (int i = 0; i < dof; i++) {
		coo.push_back(Tripletd(x_offset + i, y_offset + i, ret_stiffness));
	}
}
