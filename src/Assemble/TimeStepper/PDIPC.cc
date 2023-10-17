#include "PDIPC.hpp"

bool pdipc_registered = TimeStepperRegistration::RegisterTimeStepper<PDIPC>("PDIPC");

PDIPC::PDIPC(const json& config):
	_outer_tolerance(config["outer-tolerance"]),
	_inner_tolerance(config["inner-tolerance"]),
	_culling(Factory<CCDCulling>::GetInstance()->GetProduct(config["culling"]["type"], config["culling"])),
	_pd_ipc_collision_handler(config["collision-handler"]),
	_toi_estimator(config["toi-estimator"]) {

}

void PDIPC::BindSystem(const json &config) {
	std::vector<Object> objs;
	TypeErasure::ReadObjects(config["objects"], objs);
	BindObjects(objs.begin(), objs.end());
}

void PDIPC::BindObjects(
	const typename std::vector<Object>::const_iterator &begin,
	const typename std::vector<Object>::const_iterator &end
) {
	_coord_assembler.BindObjects(begin, end);
	_mass_assembler.BindObjects(begin, end);
	_ext_force_assembler.BindObjects(begin, end);
	_pd_assembler.BindObjects(begin, end);

	TypeErasure::Cast2Interface(begin, end, _render_objs);
	TypeErasure::Cast2Interface(begin, end, _collision_objs);
	TypeErasure::Cast2Interface(begin, end, _massed_collision_objs);

	_offsets.clear();
	_offsets.reserve(end - begin);
	int offset = 0;
	for (const auto& obj : _collision_objs) {
		_offsets.push_back(offset);
		offset += obj.GetDOF();
	}
}

void PDIPC::Step(double h) {
	const int total_dof = _coord_assembler.GetDOF();

    VectorXd x_current(total_dof), v_current(total_dof);
    _coord_assembler.GetCoordinate(x_current);
    _coord_assembler.GetVelocity(v_current);

    SparseMatrixXd M, G;
    _mass_assembler.GetMass(M);
    _pd_assembler.GetGlobalMatrix(G);
    SparseMatrixXd M_h2 = M / (h * h);

    VectorXd f_ext(total_dof);
    _ext_force_assembler.GetExternalForce(f_ext);
    
    Eigen::SimplicialLDLT<SparseMatrixXd> LDLT_solver(M);
    VectorXd x_hat = x_current + h * v_current + h * h * LDLT_solver.solve(f_ext);
    VectorXd Mx_hat_h2 = M_h2 * x_hat;

	VectorXd delta_x = x_hat - x_current;
	VectorXd x = x_hat;

	_pd_ipc_collision_handler.ClearConstraintSet();
	while (delta_x.lpNorm<Eigen::Infinity>() > _outer_tolerance) {
		VectorXd x_pre = x;
		VectorXd barrier_y = VectorXd::Zero(x.size());
		SparseMatrixXd barrier_global_matrix(total_dof, total_dof);
		double delta_E = 0;
		do {
			VectorXd y = Mx_hat_h2;
			_pd_assembler.LocalProject(x, y, 0);
			_pd_ipc_collision_handler.BarrierLocalProject(_massed_collision_objs, _offsets, y);

			COO coo;
			_pd_assembler.GetGlobalMatrix(coo, 0, 0);

			SparseMatrixXd global_matrix(total_dof, total_dof);
			global_matrix.setFromTriplets(coo.begin(), coo.end());
			global_matrix += M_h2 + barrier_global_matrix;

			LDLT_solver.compute(global_matrix);

			x = LDLT_solver.solve(y);

			delta_E = 0.5 * x.transpose() * global_matrix * x;
		} while (delta_E > _inner_tolerance);

		std::vector<PrimitivePair> constraint_set;
		_culling->GetCCDSet(_collision_objs, _offsets, constraint_set);

		VectorXd v = x - x_pre;
		_collision_assembler.ComputeCollisionVertex(x, _collision_objs);
		_collision_assembler.ComputeCollisionVertexVelocity(v, _collision_objs);

		std::vector<double> local_tois;
		double toi = _toi_estimator.GetLocalTOIs(constraint_set, _collision_objs, local_tois);

		if (toi < 1) {
			toi *= 0.8;
		}
		_pd_ipc_collision_handler.AddCollisionPairs(
			_massed_collision_objs,
			_offsets,
			constraint_set,
			local_tois,
			toi
		);
		_pd_ipc_collision_handler.BarrierLocalProject(_massed_collision_objs, _offsets, barrier_y);
		_pd_ipc_collision_handler.GetBarrierGlobalMatrix(_massed_collision_objs, _offsets, total_dof, barrier_global_matrix);

		x = x_pre + toi * (x - x_pre);
	}
}

const std::vector<Renderable>& PDIPC::GetRenderObjects() const {
	return _render_objs;
}