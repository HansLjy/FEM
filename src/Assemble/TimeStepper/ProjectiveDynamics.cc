#include "ProjectiveDynamics.hpp"
#include "spdlog/spdlog.h"

ProjectiveDynamics::ProjectiveDynamics(const json& config)
    : _max_step(config["max-step"]), _conv_tolerance(config["tolerance"]), _pd_collision_handler(config["collision-stiffness"]) {}

void ProjectiveDynamics::BindSystem(const json &config) {
	TypeErasure::ReadObjects(config["objects"], _objs);
	TypeErasure::ReadObjects(config["colliders"], _colliders);
	BindObjects(_objs.begin(), _objs.end());
	BindCollider(_colliders.begin(), _colliders.end());

	TypeErasure::Cast2Interface(_objs.begin(), _objs.end(), _render_objects);
}

void ProjectiveDynamics::BindObjects(
    const typename std::vector<Object>::const_iterator &begin,
    const typename std::vector<Object>::const_iterator &end
) {
    _coord_assembler.BindObjects(begin, end);
    _mass_assembler.BindObjects(begin, end);
    _ext_force_assembler.BindObjects(begin, end);
    _pd_assembler.BindObjects(begin, end);
	_pd_collision_handler.BindObjects(begin, end);
}

void ProjectiveDynamics::BindCollider(
	const typename std::vector<Object>::const_iterator &begin,
	const typename std::vector<Object>::const_iterator &end
) {
	_pd_collision_handler.BindColliders(begin, end);
}

void ProjectiveDynamics::Step(double h) {
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

    // LDLT_solver.compute(global_matrix);
    VectorXd x_next = x_hat;
    VectorXd x_prev(total_dof);
    int itr = 0;
	_pd_collision_handler.ComputeConstraintSet(x_hat);
    do {
        x_prev = x_next;
        VectorXd y = VectorXd::Zero(total_dof);

        _pd_assembler.LocalProject(x_next, y, 0);
		_pd_collision_handler.LocalProject(x_next, y, 0);

		COO coo;
		_pd_assembler.GetGlobalMatrix(coo, 0, 0);
		_pd_collision_handler.GetGlobalMatrix(coo, 0, 0);
		SparseMatrixXd global_matrix(total_dof, total_dof);
		global_matrix.setFromTriplets(coo.begin(), coo.end());
		global_matrix += M_h2;
		LDLT_solver.compute(global_matrix);
        x_next = LDLT_solver.solve(Mx_hat_h2 + y);
		
		itr++;
    } while ((x_prev - x_next).lpNorm<1>() > _conv_tolerance && itr <= _max_step);

    if (itr > _max_step) {
        spdlog::error("Projective Dynamics not converge");
    } else {
        spdlog::info("Projective Dynamics converge in {} steps", itr);
    }

    VectorXd v_next = (x_next - x_current) / h;
    _coord_assembler.SetCoordinate(x_next);
    _coord_assembler.SetVelocity(v_next);
}

const std::vector<Renderable> & ProjectiveDynamics::GetRenderObjects() const {
	return _render_objects;
}
