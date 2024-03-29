#include "PositionBasedPDIPC.hpp"

void PositionBasedPDIPC::DumpCoord(int itr) const {
	VectorXd x(_coord_assembler.GetDOF()), v(_coord_assembler.GetDOF());
	_coord_assembler.GetCoordinate(x);
	_coord_assembler.GetVelocity(v);
	TimeStepperUtils::DumpXV(itr, x, v);
}

void PositionBasedPDIPC::PickCoord(int itr) {
	VectorXd x, v;
	TimeStepperUtils::PickXV(itr, x, v);
	_coord_assembler.SetCoordinate(x);
	_coord_assembler.SetVelocity(v);
}

PositionBasedPDIPC* PositionBasedPDIPC::CreateFromConfig(const json &config) {
	auto culling = CCDCulling::GetProductFromConfig(config["culling"]);
	auto barrier_set_generator = BarrierSetGenerator::GetProductFromConfig(config["barrier-set-generator"]);
	return new PositionBasedPDIPC(
		config["outer-tolerance"], config["inner-tolerance"],
		config["outer-max-iterations"], config["inner-max-iterations"],
		PositionBasedPDIPCCollisionHandler(config["collision-handler"]),
		TOIEstimator(config["toi-estimator"]),
		culling,
		barrier_set_generator
	);
}

PositionBasedPDIPC::PositionBasedPDIPC (
	double outer_tolerance, double inner_tolerance,
	int outer_max_iterations, int inner_max_iterations,
	PositionBasedPDIPCCollisionHandler&& collision_handler,
	TOIEstimator&& toi_estimator,
	CCDCulling* culling,
	BarrierSetGenerator* barrier_set_generator)
	: _outer_tolerance(outer_tolerance),
	  _inner_tolerance(inner_tolerance),
	  _outer_max_itrs(outer_max_iterations),
	  _inner_max_itrs(inner_max_iterations),
	  _pb_pd_ipc_collision_handler(std::move(collision_handler)),
	  _toi_estimator(std::move(toi_estimator)),
	  _culling(culling),
	  _barrier_set_generator(barrier_set_generator) {}

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

void PositionBasedPDIPC::TotalForceZeroProject(VectorXd& force) {
	// TODO: now only support sampled objects
	const int num_points = force.size() / 3;
	Vector3d average_force = Vector3d::Zero();
	for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
		average_force += force.segment<3>(i3);
	}
	average_force /= num_points;
	for (int i = 0, i3 = 0; i < num_points; i++, i3 += 3) {
		force.segment<3>(i3) -= average_force;
	}
}

void PositionBasedPDIPC::InnerIteration(const SparseMatrixXd& lhs_out, const VectorXd& rhs_out, VectorXd& x) const {
	VectorXd rhs = rhs_out;
	_pd_assembler.LocalProject(_pd_objs, x, rhs);

	SparseMatrixXd global_matrix = _pd_assembler.GetGlobalMatrix(_pd_objs, _total_dof);
	global_matrix += lhs_out;

	Eigen::SimplicialLDLT<SparseMatrixXd> LDLT_solver;
	LDLT_solver.compute(global_matrix);
	x = LDLT_solver.solve(rhs);
}

double PositionBasedPDIPC::GetTotalEnergy(const VectorXd &x, const SparseMatrixXd &M_h2, const VectorXd &x_hat) {
	return _pd_assembler.GetEnergy(_pd_objs, x)
		   + _pb_pd_ipc_collision_handler.GetBarrierEnergy(_massed_collision_objs)
		   + 0.5 * (x - x_hat).transpose() * M_h2 * (x - x_hat);
}

VectorXd PositionBasedPDIPC::EstimateExternalForce(
	const SparseMatrixXd& M_h2,
	const VectorXd& x_current,
	const VectorXd& x_est,
	const double dt
) {
	_collision_assembler.ComputeCollisionVertex(x_current, _collision_objs);
	_collision_assembler.ComputeCollisionVertexVelocity(x_est - x_current, _collision_objs);

	std::vector<PrimitivePair> constraint_set;
	_culling->GetCCDSet(_collision_objs, _offsets, constraint_set);

	std::vector<double> local_tois;
	double toi = _toi_estimator.GetLocalTOIs(constraint_set, _collision_objs, local_tois);

	if (toi <= 1) {
		toi *= 0.8;
	} else {
		toi = 1;
	}

	// throw "fuck";

	VectorXd barrier_y = VectorXd::Zero(_total_dof);
	_pb_pd_ipc_collision_handler.BarrierLocalProject(_massed_collision_objs, _offsets, barrier_y);
	SparseMatrixXd barrier_global_matrix(_total_dof, _total_dof);
	_pb_pd_ipc_collision_handler.GetBarrierGlobalMatrix(_massed_collision_objs, _offsets, _total_dof, barrier_global_matrix);


	Eigen::SimplicialLDLT<SparseMatrixXd> collision_solver(M_h2 + barrier_global_matrix);
	// Eigen::SimplicialLDLT<SparseMatrixXd> collision_solver(M_h2 + barrier_global_matrix + A_elas);
	VectorXd rhs_vector = M_h2 * x_est + barrier_y;

	// VectorXd xc = collision_solver.solve(M_h2 * x + barrier_y);
	VectorXd xc = collision_solver.solve(rhs_vector);
	return M_h2 * (xc - x_est);
}

void PositionBasedPDIPC::Step(double h) {
    VectorXd x_current(_total_dof), v_current(_total_dof);
    _coord_assembler.GetCoordinate(x_current);
    _coord_assembler.GetVelocity(v_current);

    SparseMatrixXd M;
    _mass_assembler.GetMass(M);
    SparseMatrixXd M_h2 = M / (h * h);

    VectorXd f_ext(_total_dof);
    _ext_force_assembler.GetExternalForce(f_ext);
    
    Eigen::SimplicialLDLT<SparseMatrixXd> m_solver(M);
    VectorXd x_hat = x_current + h * v_current + h * h * m_solver.solve(f_ext);
    VectorXd Mx_hat_h2 = M_h2 * x_hat;

	_pb_pd_ipc_collision_handler.ClearConstraintSet();
	VectorXd barrier_y = VectorXd::Zero(_total_dof);
	SparseMatrixXd barrier_global_matrix(_total_dof, _total_dof);

	SparseMatrixXd mass_barrier_lhs_matrix = M_h2 + barrier_global_matrix;
	VectorXd mass_barrier_rhs_vector = Mx_hat_h2 + barrier_y;

	int B_size = 0;
	int delta_B_size = 0;
	int outer_itr = 0;
	double last_toi = 0;
	VectorXd x_ccd_start = x_current;
	VectorXd x = x_hat;							// final solution
	while(true) {
		int inner_itrs = 0;
		bool inner_converge = false;
		while(true) {
			VectorXd x_pre = x;
			InnerIteration(mass_barrier_lhs_matrix, mass_barrier_rhs_vector, x);

			double tolerance;
			if (B_size == 0 || delta_B_size <= 1) {
				tolerance = _inner_tolerance;
			} else {
				tolerance = std::log(delta_B_size) * _inner_tolerance;
			}

			if (++inner_itrs >= _inner_max_itrs || (x - x_pre).norm() < tolerance) {
				break;
			}
		}

		if (inner_itrs < _inner_max_itrs) {
			inner_converge = true;
			// spdlog::info("Inner iteration converges in {} steps", inner_itrs);
		} else {
			spdlog::warn("Inner iteration does not converge!");
		}

		_collision_assembler.ComputeCollisionVertex(x_ccd_start, _collision_objs);
		_collision_assembler.ComputeCollisionVertexVelocity(x - x_ccd_start, _collision_objs);

		std::vector<PrimitivePair> constraint_set;
		_culling->GetCCDSet(_collision_objs, _offsets, constraint_set);

		std::vector<double> local_tois;
		double toi = _toi_estimator.GetLocalTOIs(constraint_set, _collision_objs, local_tois);

		if (toi <= 1) {
			toi *= 0.8;
		} else {
			toi = 1;
		}
		last_toi = toi;
		spdlog::info("toi = {}", last_toi);

		std::vector<PrimitivePair> barrier_set;
		_collision_assembler.ComputeCollisionVertex(x_ccd_start + toi * (x - x_ccd_start), _collision_objs);
		_barrier_set_generator->GenerateBarrierSet(_collision_objs, _offsets, barrier_set);

		_pb_pd_ipc_collision_handler.AddBarrierSet(barrier_set);
		_collision_assembler.ComputeCollisionVertex(x_ccd_start, _collision_objs);
		_collision_assembler.ComputeCollisionVertexVelocity(x - x_ccd_start, _collision_objs);
		_pb_pd_ipc_collision_handler.TargetPositionGeneration(_massed_collision_objs, toi, h);

		int new_B_size = _pb_pd_ipc_collision_handler._barrier_set.size();
		delta_B_size = new_B_size - B_size;
		B_size = new_B_size;

		barrier_y.setZero();
		_pb_pd_ipc_collision_handler.BarrierLocalProject(_massed_collision_objs, _offsets, barrier_y);
		barrier_global_matrix.setZero();
		_pb_pd_ipc_collision_handler.GetBarrierGlobalMatrix(_massed_collision_objs, _offsets, _total_dof, barrier_global_matrix);

		mass_barrier_lhs_matrix = M_h2 + barrier_global_matrix;
		mass_barrier_rhs_vector = Mx_hat_h2 + barrier_y;

		const double delta_x = (x - x_ccd_start).lpNorm<Eigen::Infinity>();
		x_ccd_start += toi * (x - x_ccd_start);
		if (inner_converge && delta_x < _outer_tolerance) {
			x = x_ccd_start;
			break;
		}
		outer_itr++;
	}

	spdlog::info("Outer iteration converges in {} steps", outer_itr);

	VectorXd v = (x - x_current) / h * (1 - h);
	_coord_assembler.SetCoordinate(x);
	_coord_assembler.SetVelocity(v);
}

const std::vector<Renderable>& PositionBasedPDIPC::GetRenderObjects() const {
	return _render_objs;
}