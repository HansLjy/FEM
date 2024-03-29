#include "ProjectiveDynamics.hpp"
#include "spdlog/spdlog.h"

ProjectiveDynamics* ProjectiveDynamics::CreateFromConfig(const json& config) {
	return new ProjectiveDynamics(config["max-step"], config["tolerance"], PDCollisionHandler(static_cast<double>(config["collision-stiffness"])));
}

ProjectiveDynamics::ProjectiveDynamics(
	int max_step, double tolerance,
	PDCollisionHandler&& collision_handler)
	: _max_step(max_step), _conv_tolerance(tolerance), _pd_collision_handler(std::move(collision_handler)){}

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
	_pd_collision_handler.BindObjects(begin, end);

    TypeErasure::Cast2Interface(begin, end, _pd_objects);
    _total_dof = 0;
    for (const auto& obj : _pd_objects) {
        _total_dof += obj.GetDOF();
    }
}

void ProjectiveDynamics::BindCollider(
	const typename std::vector<Object>::const_iterator &begin,
	const typename std::vector<Object>::const_iterator &end
) {
	_pd_collision_handler.BindColliders(begin, end);
}

void ProjectiveDynamics::Step(double h) {
    VectorXd x_current(_total_dof), v_current(_total_dof);
    _coord_assembler.GetCoordinate(x_current);
    _coord_assembler.GetVelocity(v_current);

    SparseMatrixXd M;
    _mass_assembler.GetMass(M);
    SparseMatrixXd G = _pd_assembler.GetGlobalMatrix(_pd_objects, _total_dof);
    SparseMatrixXd M_h2 = M / (h * h);

    VectorXd f_ext(_total_dof);
    _ext_force_assembler.GetExternalForce(f_ext);
    
    Eigen::SimplicialLDLT<SparseMatrixXd> LDLT_solver(M);
    VectorXd x_hat = x_current + h * v_current + h * h * LDLT_solver.solve(f_ext);
    VectorXd Mx_hat_h2 = M_h2 * x_hat;

    // LDLT_solver.compute(global_matrix);
    VectorXd x = x_hat;
    VectorXd x_prev(_total_dof);
    int itr = 0;
	_pd_collision_handler.ComputeConstraintSet(x_hat);
	double energy = _pd_assembler.GetEnergy(_pd_objects, x);
    do {
        x_prev = x;
        VectorXd y = VectorXd::Zero(_total_dof);

        _pd_assembler.LocalProject(_pd_objects, x, y);
		_pd_collision_handler.LocalProject(x, y, 0);

		COO coo;
		_pd_assembler.GetGlobalMatrix(_pd_objects, coo, 0, 0);
		_pd_collision_handler.GetGlobalMatrix(coo, 0, 0);
		SparseMatrixXd global_matrix(_total_dof, _total_dof);
		global_matrix.setFromTriplets(coo.begin(), coo.end());
		global_matrix += M_h2;
		LDLT_solver.compute(global_matrix);
        x = LDLT_solver.solve(Mx_hat_h2 + y);

		energy = _pd_assembler.GetEnergy(_pd_objects, x);

		itr++;
    } while ((x_prev - x).lpNorm<Eigen::Infinity>() > _conv_tolerance && itr <= _max_step);

    if (itr > _max_step) {
        spdlog::warn("Projective Dynamics not converge");
    } else {
        spdlog::info("Projective Dynamics converge in {} steps", itr);
    }

    VectorXd v_next = (x - x_current) / h;
    _coord_assembler.SetCoordinate(x);
    _coord_assembler.SetVelocity(v_next);
}

const std::vector<Renderable> & ProjectiveDynamics::GetRenderObjects() const {
	return _render_objects;
}
