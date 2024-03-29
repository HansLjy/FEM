#include "IncrementalPotential.hpp"

IncrementalPotentialTimeStepper::~IncrementalPotentialTimeStepper() {
	delete _optimizer;
}

IncrementalPotentialTimeStepper* IncrementalPotentialTimeStepper::CreateFromConfig(const json &config) {
	bool damping_enabled = config["damping-enabled"];
	double rayleigh_coef_mass, rayleigh_coef_stiffness;
	if (damping_enabled) {
		rayleigh_coef_mass = config["rayleigh-coef-mass"];
		rayleigh_coef_stiffness = config["rayleigh-coef-stiffness"];
	}
	return new IncrementalPotentialTimeStepper(
		damping_enabled,
		rayleigh_coef_mass, rayleigh_coef_stiffness,
		Factory<Optimizer>::GetInstance()->GetProduct(config["optimizer"]["type"], config["optimizer"])
	);
}

IncrementalPotentialTimeStepper::IncrementalPotentialTimeStepper(
	double damping_enabled,
	double rayleigh_coef_mass,
	double rayleigh_coef_stiffness,
	Optimizer* optimizer)
	: _damping_enabled(damping_enabled),
	  _rayleigh_coef_mass(rayleigh_coef_mass),
	  _rayleigh_coef_stiffness(rayleigh_coef_stiffness),
	  _optimizer(optimizer) {}

void IncrementalPotentialTimeStepper::BindSystem(const json &config) {
	std::vector<Object> objs;
	TypeErasure::ReadObjects(config["obj"], objs);
	BindObjects(objs.begin(), objs.end());
	TypeErasure::Cast2Interface(objs.begin(), objs.end(), _render_objects);
}

void IncrementalPotentialTimeStepper::BindObjects(
	const typename std::vector<Object>::const_iterator &begin,
	const typename std::vector<Object>::const_iterator &end
) {
	_coord_assembler.BindObjects(begin, end);
	_mass_assembler.BindObjects(begin, end);
	_energy_assembler.BindObjects(begin, end);
	_external_force_assembler.BindObjects(begin, end);
}

void IncrementalPotentialTimeStepper::Step(double h) {
    const int dof = _coord_assembler.GetDOF();
    VectorXd x(dof), v(dof), force(dof);
    _coord_assembler.GetCoordinate(x);
    _coord_assembler.GetVelocity(v);
    _external_force_assembler.GetExternalForce(force);
    SparseMatrixXd mass;
    _mass_assembler.GetMass(mass);

	VectorXd x_hat(x.size());

	if (_damping_enabled) {
		SparseMatrixXd energy_hessian;
		_energy_assembler.GetPotentialHessian(x, energy_hessian);
		VectorXd Mv = mass * v;
		mass += h * (_rayleigh_coef_mass * mass + _rayleigh_coef_stiffness * energy_hessian);
		Eigen::SimplicialLDLT<SparseMatrixXd> LDLT_solver(mass);
		if (LDLT_solver.info() != Eigen::Success) {
			std::cerr << "System Matrix Not PSD" << std::endl;
			exit(-1);
		}
		x_hat = x + LDLT_solver.solve(h * h * force + h * Mv);
	} else {
		Eigen::SimplicialLDLT<SparseMatrixXd> LDLT_solver(mass);
		if (LDLT_solver.info() != Eigen::Success) {
			std::cerr << "System Matrix Not PSD" << std::endl;
			exit(-1);
		}
		x_hat = x + h * v + h * h * LDLT_solver.solve(force);
	}

    auto func = [&x_hat, &mass, &h, this] (const VectorXd& x) -> double {
        return 0.5 * (x - x_hat).transpose() * mass * (x - x_hat) + h * h * _energy_assembler.GetPotentialEnergy(x);
    };

    auto grad = [&x_hat, &mass, &h, &dof, this] (const VectorXd& x, VectorXd& gradient) -> void {
        if (gradient.size() != dof) {
            gradient.resize(dof);
        }
        _energy_assembler.GetPotentialGradient(x, gradient);
        gradient = h * h * gradient + mass * (x - x_hat);
    };

    auto hes = [&mass, &h, this] (const VectorXd& x, SparseMatrixXd& hessian) -> void {
        _energy_assembler.GetPotentialHessian(x, hessian);
        hessian = h * h * hessian + mass;
    };

    VectorXd x_next = x;
    _optimizer->Optimize(func, grad, hes, x_next);

	// VectorXd tmp_gradient;
	// grad(x_next, tmp_gradient);
	// spdlog::info("residue = {}", tmp_gradient.norm());

    VectorXd v_next = (x_next - x) / h;

    _coord_assembler.SetCoordinate(x_next);
    _coord_assembler.SetVelocity(v_next);

}

const std::vector<Renderable>& IncrementalPotentialTimeStepper::GetRenderObjects() const {
	return _render_objects;
}