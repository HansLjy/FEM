#include "MassSpringEnergyModel.hpp"

double MassSpringEnergyFunction::GetPotential(
	int num_edges, double stiffness, const MatrixXi& edge_topo, const VectorXd& rest_length,
	const Ref<const VectorXd> &x
) {
	double energy = 0;
	for (int i = 0; i < num_edges; i++) {
		const auto& indices = edge_topo.row(i);
		const int id1 = indices[0], id2 = indices[1];
		double delta_x = (x.segment<3>(id2 * 3) - x.segment<3>(id1 * 3)).norm() - 
		rest_length(i);
		energy += stiffness * delta_x * delta_x / 2;
	}
	return energy;
}

VectorXd MassSpringEnergyFunction::GetPotentialGradient(
	int num_edges, double stiffness, const MatrixXi& edge_topo, const VectorXd& rest_length,
	const Ref<const VectorXd> &x
) {
	VectorXd gradient = VectorXd::Zero(x.size());
	for (int i = 0; i < num_edges; i++) {
		const auto& indices = edge_topo.row(i);
		const int id1 = indices[0], id2 = indices[1];
		Vector3d e = x.segment<3>(3 * id2) - x.segment<3>(3 * id1);
		Vector3d single_gradient = - stiffness * (1 - rest_length(i) / e.norm()) * e;
		gradient.segment<3>(id1 * 3) += single_gradient;
		gradient.segment<3>(id2 * 3) += - single_gradient;
	}
	return gradient;
}


void MassSpringEnergyFunction::GetPotentialHessian(
	int num_edges, double stiffness, const MatrixXi& edge_topo, const VectorXd& rest_length,
	const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset
) {
	for (int i = 0; i < num_edges; i++) {
		const auto& indices = edge_topo.row(i);
		const int id1 = indices[0], id2 = indices[1];
		Vector3d e = x.segment<3>(3 * id2) - x.segment<3>(3 * id1);
		const double e_norm = e.norm();
		Matrix3d single_hession = stiffness * ((1 - rest_length(i) / e_norm) * Matrix3d::Identity()
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
}