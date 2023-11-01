#include "PositionBasedPDIPC.hpp"

PositionBasedPDIPC* PositionBasedPDIPC::CreateFromConfig(const json &config) {
	auto culling = CCDCulling::GetProductFromConfig(config["culling"]);
	return new PositionBasedPDIPC(
		config["outer-tolerance"], config["inner-tolerance"],
		config["outer-max-iterations"], config["inner-max-iterations"],
		PositionBasedPDIPCCollisionHandler(config["collision-handler"]),
		TOIEstimator(config["toi-estimator"]),
		culling
	);
}

PositionBasedPDIPC::PositionBasedPDIPC (
	double outer_tolerance, double inner_tolerance,
	int outer_max_iterations, int inner_max_iterations,
	PositionBasedPDIPCCollisionHandler&& collision_handler,
	TOIEstimator&& toi_estimator,
	CCDCulling* culling)
	: _outer_tolerance(outer_tolerance),
	  _inner_tolerance(inner_tolerance),
	  _outer_max_itrs(outer_max_iterations),
	  _inner_max_itrs(inner_max_iterations),
	  _pb_pd_ipc_collision_handler(std::move(collision_handler)),
	  _toi_estimator(std::move(toi_estimator)),
	  _culling(culling) {}

void PositionBasedPDIPC::BindSystem(const json &config) {
	TypeErasure::ReadObjects(config["objects"], _objs);
	BindObjects(_objs.begin(), _objs.end());
}

void PositionBasedPDIPC::BindObjects(
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

void PositionBasedPDIPC::InnerIteration(const SparseMatrixXd& lhs_out, const VectorXd& rhs_out, VectorXd& x) const {
	VectorXd rhs = rhs_out;
	_pd_assembler.LocalProject(_pd_objs, x, rhs);

	SparseMatrixXd global_matrix = _pd_assembler.GetGlobalMatrix(_pd_objs, _total_dof);
	global_matrix += lhs_out;

	Eigen::ConjugateGradient<SparseMatrixXd> CG_solver;
	CG_solver.compute(global_matrix);
	x = CG_solver.solve(rhs);
}

double PositionBasedPDIPC::GetTotalEnergy(const VectorXd &x, const SparseMatrixXd &M_h2, const VectorXd &x_hat) {
	return _pd_assembler.GetEnergy(_pd_objs, x)
		   + _pb_pd_ipc_collision_handler.GetBarrierEnergy(_massed_collision_objs)
		   + 0.5 * (x - x_hat).transpose() * M_h2 * (x - x_hat);
}

void PositionBasedPDIPC::Step(double h) {
    VectorXd x_current(_total_dof), v_current(_total_dof);
    _coord_assembler.GetCoordinate(x_current);
    _coord_assembler.GetVelocity(v_current);

    SparseMatrixXd M;
    _mass_assembler.GetMass(M);
    SparseMatrixXd G = _pd_assembler.GetGlobalMatrix(_pd_objs, _total_dof);
    SparseMatrixXd M_h2 = M / (h * h);

    VectorXd f_ext(_total_dof);
    _ext_force_assembler.GetExternalForce(f_ext);
    
    Eigen::SimplicialLDLT<SparseMatrixXd> LDLT_solver(M);
    VectorXd x_hat = x_current + h * v_current + h * h * LDLT_solver.solve(f_ext);
    VectorXd Mx_hat_h2 = M_h2 * x_hat;

	_pb_pd_ipc_collision_handler.ClearConstraintSet();
	VectorXd barrier_y = VectorXd::Zero(_total_dof);
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
			InnerIteration(M_h2 + barrier_global_matrix, Mx_hat_h2 + barrier_y, x);

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

		_collision_assembler.ComputeCollisionVertex(x_current, _collision_objs);
		_collision_assembler.ComputeCollisionVertexVelocity(x - x_current, _collision_objs);

		std::vector<PrimitivePair> constraint_set;
		_culling->GetCCDSet(_collision_objs, _offsets, constraint_set);

		std::vector<double> local_tois;
		double toi = _toi_estimator.GetLocalTOIs(constraint_set, _collision_objs, local_tois);

		if (toi <= 1) {
			toi *= 0.998;
		} else {
			toi = 1;
		}
		_pb_pd_ipc_collision_handler.AddCollisionPairs(
			_massed_collision_objs,
			_offsets,
			constraint_set,
			local_tois,
			toi
		);
		last_toi = toi;

		barrier_y.setZero();
		_pb_pd_ipc_collision_handler.BarrierLocalProject(_massed_collision_objs, _offsets, barrier_y);
		barrier_global_matrix.setZero();
		_pb_pd_ipc_collision_handler.GetBarrierGlobalMatrix(_massed_collision_objs, _offsets, _total_dof, barrier_global_matrix);

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

const std::vector<Renderable>& PositionBasedPDIPC::GetRenderObjects() const {
	return _render_objs;
}