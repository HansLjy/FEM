#include "GRAssembler.hpp"

GRAssembler::~GRAssembler() {
	delete _general_assembler;
}

void GRAssembler::BindSystem(System &system) {
	_general_assembler->BindSystem(system);
	_gr_system = dynamic_cast<GeometryReconstructSystem*>(&system);
	_triangle_offset = _gr_system->_reconstructed_mesh->GetDOF();
	assert(_gr_system);
}

int GRAssembler::GetDOF() const {
	return _general_assembler->GetDOF();
}

void GRAssembler::GetCoordinate(Ref<VectorXd> x) const {
	_general_assembler->GetCoordinate(x);
}

void GRAssembler::GetVelocity(Ref<VectorXd> v) const {
	_general_assembler->GetVelocity(v);
}

void GRAssembler::SetCoordinate(const Ref<const VectorXd> &x) {
	_general_assembler->SetCoordinate(x);
}

void GRAssembler::SetVelocity(const Ref<const VectorXd> &v) {
	_general_assembler->SetVelocity(v);
}

void GRAssembler::GetMass(COO &coo, int offset_x, int offset_y) const {
	_general_assembler->GetMass(coo, offset_x, offset_y);
}

double GRAssembler::GetPotentialEnergy(const Ref<const VectorXd> &x) const {
	double energy = _general_assembler->GetPotentialEnergy(x);
	switch (_gr_system->_status) {
		case GeometryReconstructSystem::SystemStatus::kFlying: {
			energy += GetFlyingEnergy(x);
			break;
		}
		case GeometryReconstructSystem::SystemStatus::kCrawling: {
			energy += GetCrawlingEnergy(x);
			break;
		}
		case GeometryReconstructSystem::SystemStatus::kGlueing: {
			energy += GetGlueingEnergy(x);
			break; 
		}
		case GeometryReconstructSystem::SystemStatus::kTurning: {
			// The turning energy is calculated by the grid object
			break;
		}
	}
	return energy;
}

void GRAssembler::GetPotentialEnergyGradient(const Ref<const VectorXd> &x, Ref<VectorXd> gradient) const {
	_general_assembler->GetPotentialEnergyGradient(x, gradient);
	switch (_gr_system->_status) {
		case GeometryReconstructSystem::SystemStatus::kFlying: {
			GetFlyingEnergyGradient(x, gradient);
			break;
		}
		case GeometryReconstructSystem::SystemStatus::kCrawling: {
			GetCrawlingEnergyGradient(x, gradient);
			break;
		}
		case GeometryReconstructSystem::SystemStatus::kGlueing: {
			GetGlueingEnergyGradient(x, gradient);
			break;
		}
		case GeometryReconstructSystem::SystemStatus::kTurning: {
			break;
		}
	}
}

void GRAssembler::GetPotentialEnergyHessian(const Ref<const Eigen::VectorXd> &x, COO &coo, int offset_x, int offset_y) const {
	_general_assembler->GetPotentialEnergyHessian(x, coo, offset_x, offset_y);
	switch (_gr_system->_status) {
		case GeometryReconstructSystem::SystemStatus::kFlying: {
			GetFlyingEnergyHessian(x, coo, offset_x, offset_y);
			break;
		}
		case GeometryReconstructSystem::SystemStatus::kCrawling: {
			GetCrawlingEnergyHessian(x, coo, offset_x, offset_y);
			break;
		}
		case GeometryReconstructSystem::SystemStatus::kGlueing: {
			GetGlueingEnergyHessian(x, coo, offset_x, offset_y);
			break;
		}
		case GeometryReconstructSystem::SystemStatus::kTurning: {
			break;
		}
	}
}

void GRAssembler::GetExternalForce(Ref<VectorXd> force) const {
	_general_assembler->GetExternalForce(force);
}

double GRAssembler::GetFlyingEnergy(const Ref<const VectorXd>& x) const {
	double energy = 0;
	for (int i = 0; i < 3; i++) {
		const int attach_id = _gr_system->_attach_vertex_id[i];
		if (attach_id != -1) {
			Vector3d x1 = x.segment<3>(_triangle_offset + i * 3);
			Vector3d x2 = _gr_system->_reconstructed_mesh->GetVertex(attach_id, x);
			energy += GetSpringEnergy(x2 - x1, _gr_system->_triangle_fly_stiffness);
		}
	}
	return energy;
}

void GRAssembler::GetFlyingEnergyGradient(const Ref<const VectorXd>& x, Ref<VectorXd> gradient) const {
	for (int i = 0; i < 3; i++) {
		const int attach_id = _gr_system->_attach_vertex_id[i];
		if (attach_id != -1) {
			Vector3d x1 = x.segment<3>(_triangle_offset + i * 3);
			Vector3d x2 = _gr_system->_reconstructed_mesh->GetVertex(attach_id, x);
			Vector3d local_gradient = GetSpringEnergyGradient(x2 - x1, _gr_system->_triangle_fly_stiffness);
			_gr_system->_reconstructed_mesh->GetVertexDerivative(attach_id).RightProduct(local_gradient, gradient);
			gradient.segment<3>(_triangle_offset + i * 3) -= local_gradient;
		}
	}
}

void GRAssembler::GetFlyingEnergyHessian(const Ref<const VectorXd>& x, COO& coo, int offset_x, int offset_y) const {
	for (int i = 0, i3 = 0; i < 3; i++, i3 += 3) {
		const int attach_id = _gr_system->_attach_vertex_id[i];
		if (attach_id != -1) {
			Vector3d x1 = x.segment<3>(_triangle_offset + i * 3);
			Vector3d x2 = _gr_system->_reconstructed_mesh->GetVertex(attach_id, x);
			Matrix3d local_hessian = GetSpringEnergyHessian(x2 - x1, _gr_system->_triangle_fly_stiffness);
			Matrix6d local_hessian_full;
			local_hessian_full.block<3, 3>(0, 0) = local_hessian_full.block<3, 3>(3, 3) = local_hessian;
			local_hessian_full.block<3, 3>(0, 3) = local_hessian_full.block<3, 3>(3, 0) = -local_hessian;
			local_hessian_full = PositiveProject(local_hessian_full);

			const int offsets[2] = {0, _triangle_offset};
			const BlockVector object_gradient[] = {
				_gr_system->_reconstructed_mesh->GetVertexDerivative(attach_id),
				_gr_system->_new_triangle->GetVertexDerivative(i)
			};

			for (int j = 0, j3 = 0; j < 2; j++, j3 += 3) {
				for (int k = 0, k3 = 0; k < 2; k++, k3 += 3) {
					object_gradient[j].RightProduct(local_hessian_full.block<3, 3>(j3, k3))
					.RightTransposeProduct(object_gradient[k])
					.ToSparse(coo, offset_x + offsets[j], offset_y + offsets[k]);
				}
			}
		}
	}
}

double GRAssembler::GetGlueingEnergy(const Ref<const VectorXd> &x) const {
	double energy = 0;
	for (int i = 0; i < 3; i++) {
		const int glue_id = _gr_system->_attach_vertex_id[i];
		if (glue_id != -1) {
			Vector3d x1 = x.segment<3>(_triangle_offset + i * 3);
			Vector3d x2 = _gr_system->_reconstructed_mesh->GetVertex(glue_id, x);
			energy += GetSpringEnergy(x2 - x1, _gr_system->_triangle_glue_stiffness);
		}
	}
	return energy;
}

void GRAssembler::GetGlueingEnergyGradient(const Ref<const VectorXd> &x, Ref<VectorXd> gradient) const {
	for (int i = 0; i < 3; i++) {
		const int glue_id = _gr_system->_attach_vertex_id[i];
		if (glue_id != -1) {
			Vector3d x1 = x.segment<3>(_triangle_offset + i * 3);
			Vector3d x2 = _gr_system->_reconstructed_mesh->GetVertex(glue_id, x);
			Vector3d local_gradient = GetSpringEnergyGradient(x2 - x1, _gr_system->_triangle_glue_stiffness);
			_gr_system->_reconstructed_mesh->GetVertexDerivative(glue_id).RightProduct(local_gradient, gradient);
			gradient.segment<3>(_triangle_offset + i * 3) -= local_gradient;
		}
	}
}

void GRAssembler::GetGlueingEnergyHessian(const Ref<const VectorXd> &x, COO &coo, int offset_x, int offset_y) const {
	for (int i = 0, i3 = 0; i < 3; i++, i3 += 3) {
		const int glue_id = _gr_system->_attach_vertex_id[i];
		if (glue_id != -1) {
			Vector3d x1 = x.segment<3>(_triangle_offset + i * 3);
			Vector3d x2 = _gr_system->_reconstructed_mesh->GetVertex(glue_id, x);
			Matrix3d local_hessian = GetSpringEnergyHessian(x2 - x1, _gr_system->_triangle_glue_stiffness);
			Matrix6d local_hessian_full;
			local_hessian_full.block<3, 3>(0, 0) = local_hessian_full.block<3, 3>(3, 3) = local_hessian;
			local_hessian_full.block<3, 3>(0, 3) = local_hessian_full.block<3, 3>(3, 0) = -local_hessian;
			local_hessian_full = PositiveProject(local_hessian_full);

			const int offsets[2] = {0, _triangle_offset};
			const BlockVector object_gradient[] = {
				_gr_system->_reconstructed_mesh->GetVertexDerivative(glue_id),
				_gr_system->_new_triangle->GetVertexDerivative(i)
			};

			for (int j = 0, j3 = 0; j < 2; j++, j3 += 3) {
				for (int k = 0, k3 = 0; k < 2; k++, k3 += 3) {
					object_gradient[j].RightProduct(local_hessian_full.block<3, 3>(j3, k3))
					.RightTransposeProduct(object_gradient[k])
					.ToSparse(coo, offset_x + offsets[j], offset_y + offsets[k]);
				}
			}
		}
	}
}

double GRAssembler::GetCrawlingEnergy(const Ref<const VectorXd> &x) const {
	double energy = 0;
	for (int i = 0; i < 3; i++) {
		energy += GetSpringEnergy(_gr_system->_local_target_position - x.segment<3>(_triangle_offset + i * 3), _gr_system->_triangle_crawl_stiffness);
	}
	return energy;
}

void GRAssembler::GetCrawlingEnergyGradient(const Ref<const VectorXd> &x, Ref<VectorXd> gradient) const {
	for (int i = 0, i3 = 0; i < 3; i++, i3 += 3) {
		gradient.segment<3>(_triangle_offset + i3) += GetSpringEnergyGradient(
			_gr_system->_local_target_position - x.segment<3>(_triangle_offset + i3),
			_gr_system->_triangle_crawl_stiffness
		);
	}
}


void GRAssembler::GetCrawlingEnergyHessian(const Ref<const VectorXd> &x, COO &coo, int offset_x, int offset_y) const {
	for (int i = 0, i3 = 0; i < 3; i++, i3 += 3) {
		Matrix3d local_hessian = GetSpringEnergyHessian(
			x.segment<3>(_triangle_offset + i3) - _gr_system->_local_target_position,
			_gr_system->_crawling_distance
		);
		local_hessian = PositiveProject(local_hessian);
		for (int j = 0; j < 3; j++) {
			for (int k = 0; k < 3; k++) {
				coo.push_back(Tripletd(
					offset_x + _triangle_offset + i3 + j,
					offset_y + _triangle_offset + i3 + k,
					local_hessian(j, k)
				));
			}
		}
	}
}

double GRAssembler::GetSpringEnergy(const Vector3d& e, double stiffness) const {
	return 0.5 * stiffness * e.squaredNorm();
}

Vector3d GRAssembler::GetSpringEnergyGradient(const Vector3d& e, double stiffness) const {
	return stiffness * e;
}

Matrix3d GRAssembler::GetSpringEnergyHessian(const Vector3d& e, double stiffness) const {
	return stiffness * Matrix3d::Identity();
}