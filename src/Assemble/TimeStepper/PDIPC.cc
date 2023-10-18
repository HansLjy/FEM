#include "PDIPC.hpp"

PDIPC::PDIPC(const json& config):
	_outer_tolerance(config["outer-tolerance"]),
	_inner_tolerance(config["inner-tolerance"]),
	_outer_max_itrs(config["outer-max-iterations"]),
	_inner_max_itrs(config["inner-max-iterations"]),
	_culling(Factory<CCDCulling>::GetInstance()->GetProduct(config["culling"]["type"], config["culling"])),
	_pd_ipc_collision_handler(config["collision-handler"]),
	_toi_estimator(config["toi-estimator"]) {

}

void PDIPC::BindSystem(const json &config) {
	TypeErasure::ReadObjects(config["objects"], _objs);
	BindObjects(_objs.begin(), _objs.end());
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

	VectorXd x = x_hat;
	VectorXd x_pre = x_current;
	VectorXd delta_x(x.size());

	static int global_itrs = 0;
	global_itrs++;
	_pd_ipc_collision_handler.ClearConstraintSet();
	VectorXd barrier_y = VectorXd::Zero(x.size());
	SparseMatrixXd barrier_global_matrix(total_dof, total_dof);

	Eigen::ConjugateGradient<SparseMatrixXd> CG_solver;

	int outer_itr = 0;
	do {
		VectorXd inner_delta_x;
		VectorXd inner_pre_x = x;
		int inner_itrs = 0;
		do {
			VectorXd y = Mx_hat_h2 + barrier_y;
			_pd_assembler.LocalProject(x, y, 0);

			COO coo;
			_pd_assembler.GetGlobalMatrix(coo, 0, 0);

			SparseMatrixXd global_matrix(total_dof, total_dof);
			global_matrix.setFromTriplets(coo.begin(), coo.end());
			global_matrix += M_h2 + barrier_global_matrix;

			CG_solver.compute(global_matrix);
			// LDLT_solver.compute(global_matrix);

			x = CG_solver.solve(y);
			// x = LDLT_solver.solve(y);

			inner_delta_x = x - inner_pre_x;
			inner_pre_x = x;
			inner_itrs++;
		} while (inner_delta_x.lpNorm<Eigen::Infinity>() > _inner_tolerance && inner_itrs < _inner_max_itrs);

		// if (inner_itrs < _inner_max_itrs) {
		// 	spdlog::info("Inner iteration converges in {} steps", inner_itrs);
		// } else {
		// 	spdlog::warn("Inner iteration does not converge!");
		// }

		delta_x = x - x_pre;
		_collision_assembler.ComputeCollisionVertex(x_pre, _collision_objs);
		_collision_assembler.ComputeCollisionVertexVelocity(delta_x, _collision_objs);

		std::vector<PrimitivePair> constraint_set;
		_culling->GetCCDSet(_collision_objs, _offsets, constraint_set);

		std::vector<double> local_tois;
		double toi = _toi_estimator.GetLocalTOIs(constraint_set, _collision_objs, local_tois);
		// std::cerr << __FILE__ << ":" << __LINE__ << std::endl;
		// std::cerr << toi << std::endl;

		// std::cerr << __FILE__ << ":" << __LINE__ << std::endl;
		// bool flag = false;
		// for (int i = 0; i < local_tois.size(); i++) {
		// 	if (local_tois[i] <= 1) {
		// 		PrintPrimitivePair(constraint_set[i]);
		// 		std::cerr << _collision_objs[constraint_set[i]._obj_id1].GetCollisionEdgeTopo().row(constraint_set[i]._primitive_id1) << std::endl;
		// 		std::cerr << _collision_objs[constraint_set[i]._obj_id2].GetCollisionEdgeTopo().row(constraint_set[i]._primitive_id2) << std::endl;
		// 		std::cerr << local_tois[i] << std::endl;
		// 		flag = true;
		// 	}
		// }
		// if (flag) {
		// 	exit(-1);
		// }

		// int num_collision_pair = 0;
		// for (const auto& local_toi : local_tois) {
		// 	num_collision_pair += (local_toi <= 1);
		// }
		// spdlog::info("#cancidates = {}, #collision pairs = {}", constraint_set.size(), num_collision_pair);

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

		barrier_y.setZero();
		_pd_ipc_collision_handler.BarrierLocalProject(_massed_collision_objs, _offsets, barrier_y);
		barrier_global_matrix.setZero();
		_pd_ipc_collision_handler.GetBarrierGlobalMatrix(_massed_collision_objs, _offsets, total_dof, barrier_global_matrix);

		x = x_pre + toi * (x - x_pre);
		x_pre = x;

		outer_itr++;
	} while (delta_x.lpNorm<Eigen::Infinity>() > _outer_tolerance && outer_itr < _outer_max_itrs);

	if (outer_itr < _outer_max_itrs) {
		spdlog::info("Outer iteration converges in {} steps", outer_itr);
	} else {
		spdlog::warn("Outer iteration does not converge!");
	}
	spdlog::info("Global itrs = {}", global_itrs);

	// std::cerr << __FILE__ << ":" << __LINE__ << std::endl;
	// std::cerr << x.transpose() << std::endl;

	VectorXd v = (x - x_current) / h;
	_coord_assembler.SetCoordinate(x);
	_coord_assembler.SetVelocity(v);
}

const std::vector<Renderable>& PDIPC::GetRenderObjects() const {
	return _render_objs;
}