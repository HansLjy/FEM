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

    SparseMatrixXd M, G;
    _mass_assembler.GetMass(M);
    _pd_assembler.GetGlobalMatrix(_pd_objects, _total_dof, G);
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
	std::cerr << __FILE__ << ":" << __LINE__ << std::endl;
	std::fstream energy_file("./energy.txt", std::fstream::app | std::fstream::out);
	double energy = _pd_assembler.GetEnergy(_pd_objects, x);
	energy_file << energy;
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
		energy_file << ", " << energy;

		itr++;
    } while ((x_prev - x).lpNorm<1>() > _conv_tolerance && itr <= _max_step);
	energy_file << std::endl;
	energy_file.close();

    if (itr > _max_step) {
        spdlog::error("Projective Dynamics not converge");
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
