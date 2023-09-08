#pragma once
#include "System/GRSystem.hpp"
#include "GeneralAssembler.hpp"

class GRAssembler : public Assembler {
public:
	void BindSystem(System &system) override;
	int GetDOF() const override;
	
	void GetCoordinate(Ref<VectorXd> x) const override;
	void GetVelocity(Ref<VectorXd> v) const override;

	void SetCoordinate(const Ref<const VectorXd> &x) override;
	void SetVelocity(const Ref<const VectorXd> &v) override;

	void GetMass(SparseMatrixXd &mass) const override;
	void GetMass(COO &coo, int offset_x, int offset_y) const override;

	double GetPotentialEnergy(const Ref<const VectorXd> &x) const override;
	void GetPotentialEnergyGradient(const Ref<const VectorXd> &x, Ref<VectorXd> gradient) const override;
	void GetPotentialEnergyHessian(const Ref<const VectorXd> &x, SparseMatrixXd &hessian) const override;
	void GetPotentialEnergyHessian(const Ref<const Eigen::VectorXd> &x, COO &coo, int offset_x, int offset_y) const override;

	void GetExternalForce(Ref<VectorXd> force) const override;

	~GRAssembler();
	
protected:
	GeneralAssembler* _general_assembler = nullptr;
};