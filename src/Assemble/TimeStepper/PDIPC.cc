#include "PDIPC.hpp"

PDIPC* PDIPC::CreateFromConfig(const json &config) {
	return new PDIPC (
		config["outer-tolerance"],
		config["inner-tolerance"],
		config["outer-max-iterations"],
		config["inner-max-iterations"],
		PDIPCCollisionHandler(config["collision-handler"]),
		TOIEstimator(config["toi-estimator"]),
		CCDCulling::GetProductFromConfig(config["culling"]),
		BarrierSetGenerator::GetProductFromConfig(config["barrier-set-generator"])
	);	
}

PDIPC::PDIPC(
	double outer_tolerance, double inner_tolerance,
	int outer_max_itrs, int inner_max_itrs,
	PDIPCCollisionHandler&& collision_handler,
	TOIEstimator&& toi_estimator,
	CCDCulling* culling,
	BarrierSetGenerator* barrier_set_generator)
	: _outer_tolerance(outer_tolerance),
	  _inner_tolerance(inner_tolerance),
	  _outer_max_itrs(outer_max_itrs),
	  _inner_max_itrs(inner_max_itrs),
	  _pd_ipc_collision_handler(std::move(collision_handler)),
	  _toi_estimator(std::move(toi_estimator)),
	  _culling(culling),
	  _barrier_set_generator(barrier_set_generator)
	 {}

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
	TypeErasure::Cast2Interface(begin, end, _pd_objs);

	_offsets.clear();
	_offsets.reserve(end - begin);
	_total_dof = 0;
	for (const auto& obj : _collision_objs) {
		_offsets.push_back(_total_dof);
		_total_dof += obj.GetDOF();
	}
}

void PDIPC::InnerIteration(const SparseMatrixXd& lhs_out, const VectorXd& rhs_out, VectorXd& x) {
	VectorXd rhs = rhs_out;
	_pd_assembler.LocalProject(_pd_objs, x, rhs);
	_collision_assembler.ComputeCollisionVertex(x, _collision_objs);
	_pd_ipc_collision_handler.BarrierLocalProject(_collision_objs, _offsets, rhs);

	SparseMatrixXd global_matrix = _pd_assembler.GetGlobalMatrix(_pd_objs, _total_dof);
	global_matrix += lhs_out;

	Eigen::ConjugateGradient<SparseMatrixXd> CG_solver;
	CG_solver.compute(global_matrix);
	x = CG_solver.solve(rhs);
}

double PDIPC::GetTotalEnergy(const VectorXd &x, const SparseMatrixXd &M_h2, const VectorXd &x_hat) {
	return _pd_assembler.GetEnergy(_pd_objs, x)
		   + _pd_ipc_collision_handler.GetBarrierEnergy(_collision_objs)
		   + 0.5 * (x - x_hat).transpose() * M_h2 * (x - x_hat);
}

void PDIPC::Step(double h) {
    VectorXd x_current(_total_dof), v_current(_total_dof);
    _coord_assembler.GetCoordinate(x_current);
    _coord_assembler.GetVelocity(v_current);

    SparseMatrixXd M;
    _mass_assembler.GetMass(M);
    SparseMatrixXd M_h2 = M / (h * h);

    VectorXd f_ext(_total_dof);
    _ext_force_assembler.GetExternalForce(f_ext);
    
    Eigen::SimplicialLDLT<SparseMatrixXd> LDLT_solver(M);
    VectorXd x_hat = x_current + h * v_current + h * h * LDLT_solver.solve(f_ext);
    VectorXd Mx_hat_h2 = M_h2 * x_hat;

	_pd_ipc_collision_handler.ClearBarrierSet();
	SparseMatrixXd barrier_global_matrix(_total_dof, _total_dof);

	static int global_itrs = 0;
	global_itrs++;

	Eigen::ConjugateGradient<SparseMatrixXd> CG_solver;

	int outer_itr = 0;
	double last_toi = 0;
	VectorXd x = x_hat;							// final solution
	do {
		x = x_hat;

		int inner_itrs = 0;
		_collision_assembler.ComputeCollisionVertex(x, _collision_objs);
		double E_prev = GetTotalEnergy(x, M_h2, x_hat);
		double E_delta = 0;
		do {
			InnerIteration(M_h2 + barrier_global_matrix, Mx_hat_h2, x);

			_collision_assembler.ComputeCollisionVertex(x, _collision_objs);
			double E_current = GetTotalEnergy(x, M_h2, x_hat);
			E_delta = E_prev > 1e-6
					? std::abs(E_current - E_prev) / E_prev
					: std::abs(E_current - E_prev);
			E_prev = E_current;

			inner_itrs++;
		} while (E_delta > _inner_tolerance && inner_itrs < _inner_max_itrs);


		if (inner_itrs < _inner_max_itrs) {
			// spdlog::info("Inner iteration converges in {} steps", inner_itrs);
		} else {
			spdlog::warn("Inner iteration does not converge!");
		}

		std::vector<PrimitivePair> constraint_set;
		_collision_assembler.ComputeCollisionVertex(x_current, _collision_objs);
		_collision_assembler.ComputeCollisionVertexVelocity(x - x_current, _collision_objs);
		_culling->GetCCDSet(_collision_objs, _offsets, constraint_set);

		std::vector<double> local_tois;
		double toi = _toi_estimator.GetLocalTOIs(constraint_set, _collision_objs, local_tois);

		if (toi <= 1) {
			toi *= 0.998;
		} else {
			toi = 1;
		}

		std::vector<PrimitivePair> barrier_candidate_set;
		_collision_assembler.ComputeCollisionVertex(x_current + toi * (x - x_current), _collision_objs);
		_barrier_set_generator->GenerateBarrierSet(_collision_objs, _offsets, barrier_candidate_set);
		_pd_ipc_collision_handler.AddBarrierPairs(_collision_objs, _offsets, barrier_candidate_set);

		last_toi = toi;

		barrier_global_matrix = _pd_ipc_collision_handler.GetBarrierGlobalMatrix(
			_collision_objs, _offsets, _total_dof
		);

		x = x_current + toi * (x - x_current);
		outer_itr++;
	} while (outer_itr <= 1 || last_toi < 0.9 && outer_itr < _outer_max_itrs);

	if (last_toi < 0.9) {
		spdlog::warn("TOI = {}", last_toi);
	}

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