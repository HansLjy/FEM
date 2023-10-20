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

	TypeErasure::Cast2Interface(begin, end, _render_objs);
	TypeErasure::Cast2Interface(begin, end, _collision_objs);
	TypeErasure::Cast2Interface(begin, end, _massed_collision_objs);
	TypeErasure::Cast2Interface(begin, end, _pd_objs);

	_offsets.clear();
	_offsets.reserve(end - begin);
	_total_dof = 0;
	for (const auto& obj : _collision_objs) {
		_offsets.push_back(_total_dof);
		_total_dof += obj.GetDOF();
	}
}

void PDIPC::Step(double h) {
    VectorXd x_current(_total_dof), v_current(_total_dof);
    _coord_assembler.GetCoordinate(x_current);
    _coord_assembler.GetVelocity(v_current);

    SparseMatrixXd M, G;
    _mass_assembler.GetMass(M);
    _pd_assembler.GetGlobalMatrix(_pd_objs, _total_dof, G);
    SparseMatrixXd M_h2 = M / (h * h);

    VectorXd f_ext(_total_dof);
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
	SparseMatrixXd barrier_global_matrix(_total_dof, _total_dof);

	Eigen::ConjugateGradient<SparseMatrixXd> CG_solver;

	int outer_itr = 0;

	std::cerr << __FILE__ << ":" << __LINE__ << std::endl;
	std::fstream toi_file("./toi.txt", std::fstream::out | std::fstream::app);

	std::cerr << __FILE__ << ":" << __LINE__ << std::endl;
	std::fstream delta_x_file("./delta-x.txt", std::fstream::out | std::fstream::app);

	do {
		int inner_itrs = 0;

		_collision_assembler.ComputeCollisionVertex(x, _collision_objs);
		double E_prev = _pd_assembler.GetEnergy(_pd_objs, x)
					  + _pd_ipc_collision_handler.GetBarrierEnergy(_massed_collision_objs)
					  + 0.5 * (x - x_hat).transpose() * M_h2 * (x - x_hat);
		double E_delta = 0;
		VectorXd inner_delta_x;
		VectorXd inner_pre_x = x;
		do {
			VectorXd y = Mx_hat_h2 + barrier_y;
			_pd_assembler.LocalProject(_pd_objs, x, y);

			COO coo;
			_pd_assembler.GetGlobalMatrix(_pd_objs, coo, 0, 0);

			SparseMatrixXd global_matrix(_total_dof, _total_dof);
			global_matrix.setFromTriplets(coo.begin(), coo.end());
			global_matrix += M_h2 + barrier_global_matrix;

			CG_solver.compute(global_matrix);
			// LDLT_solver.compute(global_matrix);

			x = CG_solver.solve(y);
			// x = LDLT_solver.solve(y);

			_collision_assembler.ComputeCollisionVertex(x, _collision_objs);
			double E_current = _pd_assembler.GetEnergy(_pd_objs, x)
							 + _pd_ipc_collision_handler.GetBarrierEnergy(_massed_collision_objs)
					  		 + 0.5 * (x - x_hat).transpose() * M_h2 * (x - x_hat);
			// energy_file << ", " << E_current;
			E_delta = E_current - E_prev;
			E_prev = E_current;

			inner_delta_x = x - inner_pre_x;
			inner_pre_x = x;
			inner_itrs++;


		} while (inner_delta_x.lpNorm<Eigen::Infinity>() > _inner_tolerance && inner_itrs < _inner_max_itrs);
		// energy_file << std::endl;
		delta_x_file << inner_delta_x.lpNorm<Eigen::Infinity>() << ", ";

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

		if (toi <= 1) {
			toi *= 0.8;
		} else {
			toi = 1;
		}
		toi_file << toi << ", ";

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
		_pd_ipc_collision_handler.GetBarrierGlobalMatrix(_massed_collision_objs, _offsets, _total_dof, barrier_global_matrix);

		x = x_pre + toi * (x - x_pre);
		x_pre = x;

		outer_itr++;
	} while (delta_x.lpNorm<Eigen::Infinity>() > _outer_tolerance && outer_itr < _outer_max_itrs);
	toi_file << std::endl;
	toi_file.close();
	delta_x_file << std::endl;
	delta_x_file.close();

	if (outer_itr < _outer_max_itrs) {
		spdlog::info("Outer iteration converges in {} steps", outer_itr);
	} else {
		spdlog::warn("Outer iteration does not converge!");
	}
	spdlog::info("Global itrs = {}", global_itrs);

	VectorXd v = (x - x_current) / h;
	_coord_assembler.SetCoordinate(x);
	_coord_assembler.SetVelocity(v);
}

const std::vector<Renderable>& PDIPC::GetRenderObjects() const {
	return _render_objs;
}