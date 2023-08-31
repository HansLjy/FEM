#include "Physics.hpp"

class GridPhysics : public SampledPhysics {
public:
	GridPhysics() = default;
	explicit GridPhysics(const json& config) {}

	template<class Data> static void SetCoordinate(Data* obj, const Ref<const VectorXd>& x);

	template<class Data> double GetPotential(const Data* data, const Ref<const VectorXd> &x) const;
	template<class Data> VectorXd GetPotentialGradient(const Data* data, const Ref<const VectorXd> &x) const;
	template<class Data> void GetPotentialHessian(const Data* data, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const;
};

template<class Data> void GridPhysics::SetCoordinate(Data *obj, const Ref<const VectorXd> &x) {
	obj->_x.head(obj->_dof) = x;
	obj->UpdateProxyPosition();
}

template<class Data>
double GridPhysics::GetPotential(const Data *data, const Ref<const VectorXd> &x) const {
	const int num_edges = data->_num_edges;
	const int start_diag_edge = data->_start_diag_edges;
	const int num_points = data->_num_points;
	const double stiffness = data->_stiffness;
	const double diag_stiffness = data->_diag_stiffness;
	const double ret_stiffness = data->_ret_stiffness;
	const MatrixXi& edge_topo = data->_edge_topo;
	const VectorXd& rest_length = data->_rest_length;
	const VectorXd& x_rest = data->_x_rest;

	double energy = 0;
	for (int i = 0; i < start_diag_edge; i++) {
		const auto& indices = edge_topo.row(i);
		const int id1 = indices[0], id2 = indices[1];
		double delta_x = (x.segment<3>(id2 * 3) - x.segment<3>(id1 * 3)).norm() - 
		rest_length(i);
		energy += stiffness * delta_x * delta_x / 2;
	}

	for (int i = start_diag_edge; i < num_edges; i++) {
		const auto& indices = edge_topo.row(i);
		const int id1 = indices[0], id2 = indices[1];
		double delta_x = (x.segment<3>(id2 * 3) - x.segment<3>(id1 * 3)).norm() - 
		rest_length(i);
		energy += diag_stiffness * delta_x * delta_x / 2;
	}

	const std::vector<bool>& fix_points = data->_fix_points;
	for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
		if (fix_points[i]) {
			double delta_x = (x.segment<3>(i3) - x_rest.segment<3>(i3)).norm();
			energy += ret_stiffness * delta_x * delta_x / 2;
		}
	}
	return energy;
}

template<class Data>
VectorXd GridPhysics::GetPotentialGradient(const Data* data, const Ref<const VectorXd> &x) const {
	const int num_points = data->_num_points;
	const int start_diag_edge = data->_start_diag_edges;
	const int num_edges = data->_num_edges;
	const double stiffness = data->_stiffness;
	const double diag_stiffness = data->_diag_stiffness;
	const double ret_stiffness = data->_ret_stiffness;
	const MatrixXi& edge_topo = data->_edge_topo;
	const VectorXd& rest_length = data->_rest_length;
	const VectorXd& x_rest = data->_x_rest;

	VectorXd gradient = VectorXd::Zero(x.size());
	for (int i = 0; i < start_diag_edge; i++) {
		const auto& indices = edge_topo.row(i);
		const int id1 = indices[0], id2 = indices[1];
		Vector3d e = x.segment<3>(3 * id2) - x.segment<3>(3 * id1);
		Vector3d single_gradient = - stiffness * (1 - rest_length(i) / e.norm()) * e;
		gradient.segment<3>(id1 * 3) += single_gradient;
		gradient.segment<3>(id2 * 3) += - single_gradient;
	}

	for (int i = start_diag_edge; i < num_edges; i++) {
		const auto& indices = edge_topo.row(i);
		const int id1 = indices[0], id2 = indices[1];
		Vector3d e = x.segment<3>(3 * id2) - x.segment<3>(3 * id1);
		Vector3d single_gradient = - diag_stiffness * (1 - rest_length(i) / e.norm()) * e;
		gradient.segment<3>(id1 * 3) += single_gradient;
		gradient.segment<3>(id2 * 3) += - single_gradient;
	}

	const std::vector<bool>& fix_points = data->_fix_points;
	for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
		if (fix_points[i]) {
			Vector3d e = x.segment<3>(i3) - x_rest.segment<3>(i3);
			gradient.segment<3>(i3) += ret_stiffness * e;
		}
	}

	return gradient;
}

template<class Data>
void GridPhysics::GetPotentialHessian(const Data* data, const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const {
	const int dof = data->_dof;
	const int num_edges = data->_num_edges;
	const int start_diag_edge = data->_start_diag_edges;
	const double stiffness = data->_stiffness;
	const double diag_stiffness = data->_diag_stiffness;
	const double ret_stiffness = data->_ret_stiffness;
	const MatrixXi& edge_topo = data->_edge_topo;
	const VectorXd& rest_length = data->_rest_length;

	for (int i = 0; i < num_edges; i++) {
		const auto& indices = edge_topo.row(i);
		const int id1 = indices[0], id2 = indices[1];
		Vector3d e = x.segment<3>(3 * id2) - x.segment<3>(3 * id1);
		const double e_norm = e.norm();
		double cur_stiffness = i < start_diag_edge ? stiffness : diag_stiffness;
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

	const std::vector<bool>& fix_points = data->_fix_points;
	for (int i = 0; i < dof; i++) {
		if (fix_points[i / 3]) {
			coo.push_back(Tripletd(x_offset + i, y_offset + i, ret_stiffness));
		}
	}
}

