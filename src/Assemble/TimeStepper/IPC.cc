
#include "IPC.hpp"

IPC::IPC(const json& config)
: _assembler(Factory<Assembler>::GetInstance()->GetProduct(config["assembler"]["type"], config["assembler"])),
  _helper(new IPCHelper(config["ipc-helper"])),
  _max_iter(config["max-iteration"]), _tolerance(config["tolerance"]) {}

IPC::~IPC() {
	delete _helper;
}

void IPC::Step(double h) {
	_assembler->BindSystem(*(this->_system));

	const int dof = _assembler->GetDOF();
	const auto& objs = _system->GetObjs();
	_helper->SetObjects<Object>(objs.begin(), objs.end());

    VectorXd x(dof), v(dof), force(dof);
    _assembler->GetCoordinate(x);
    _assembler->GetVelocity(v);
    _assembler->GetExternalForce(force);

    SparseMatrixXd mass;
    _assembler->GetMass(mass);

    Eigen::SimplicialLDLT<SparseMatrixXd> LDLT_solver(mass);
    VectorXd a = LDLT_solver.solve(force);

    VectorXd x_hat = x + h * v + h * h * a;

	double residue;
    int step = 0;
    SparseMatrixXd hessian;
    VectorXd gradient(dof), x_prev(x);
	static int fuck_itr = 0;
    fuck_itr++;
    while(step++ < _max_iter) {
        _helper->ComputeConstraintSet(x);

		COO coo;
		hessian.resize(dof, dof);
        _assembler->GetPotentialEnergyHessian(x, coo, 0, 0);
		_helper->GetBarrierEnergyHessian(coo, 0, 0);
		hessian.setFromTriplets(coo.begin(), coo.end());

        _assembler->GetPotentialEnergyGradient(x, gradient);
		gradient += _helper->GetBarrierEnergyGradient();
        double prev_energy = h * h * (_assembler->GetPotentialEnergy(x) + _helper->GetBarrierEnergy()) + 0.5 * (x - x_hat).transpose() * mass * (x - x_hat);

        LDLT_solver.compute(mass + h * h * hessian);
        VectorXd p = - LDLT_solver.solve(mass * (x - x_hat) + h * h * gradient);

        if ((residue = p.lpNorm<Eigen::Infinity>()) < _tolerance * h) {
            break;
        }

        /* contact aware line search */
        double alpha = _helper->GetMaxStep(p);
        // spdlog::info("Max step: {}", alpha);
        VectorXd x_next;
        while (true) {
            x_next = x + alpha * p;
            _helper->ComputeConstraintSet(x_next);
            auto current_energy = h * h * (_assembler->GetPotentialEnergy(x_next) + _helper->GetBarrierEnergy()) + 0.5 * (x_next - x_hat).transpose() * mass * (x_next - x_hat);
            // spdlog::info("Current energy: {}, Previous energy: {}", current_energy, prev_energy);
            if (current_energy <= prev_energy) {
                break;
            }
            alpha *= 0.5;
        }
        // spdlog::info("alpha = {}", alpha);
        x = x_next;
    }
    // spdlog::info("Itr: {}", fuck_itr);
    if (step >= _max_iter) {
        spdlog::warn("Barrier-aware Newton not converge, residue = {}", residue);
        // exit(-1);
    } else {
        // spdlog::info("Barrier-aware Newton converge in {} steps", step);
    }

	// std::cerr << x.transpose() << std::endl;
    _assembler->SetCoordinate(x);
    _assembler->SetVelocity((x - x_prev) / h);
}