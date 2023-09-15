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

	void GetMass(COO &coo, int offset_x, int offset_y) const override;

	double GetPotentialEnergy(const Ref<const VectorXd> &x) const override;
	void GetPotentialEnergyGradient(const Ref<const VectorXd> &x, Ref<VectorXd> gradient) const override;
	void GetPotentialEnergyHessian(const Ref<const Eigen::VectorXd> &x, COO &coo, int offset_x, int offset_y) const override;

	void GetExternalForce(Ref<VectorXd> force) const override;

	~GRAssembler();

	double GetFlyingEnergy(const Ref<const VectorXd>& x) const;
	void GetFlyingEnergyGradient(const Ref<const VectorXd>& x, Ref<VectorXd> gradient) const;
	void GetFlyingEnergyHessian(const Ref<const VectorXd>& x, COO& coo, int offset_x, int offset_y) const;
	
	double GetGlueingEnergy(const Ref<const VectorXd>& x) const;
	void GetGlueingEnergyGradient(const Ref<const VectorXd>& x, Ref<VectorXd> gradient) const;
	void GetGlueingEnergyHessian(const Ref<const VectorXd>& x, COO& coo, int offset_x, int offset_y) const;

	double GetCrawlingEnergy(const Ref<const VectorXd>& x) const;
	void GetCrawlingEnergyGradient(const Ref<const VectorXd>& x, Ref<VectorXd> gradient) const;
	void GetCrawlingEnergyHessian(const Ref<const VectorXd>& x, COO& coo, int offset_x, int offset_y) const;

protected:
	GeneralAssembler* _general_assembler = nullptr;
	GeometryReconstructSystem* _gr_system = nullptr;
	int _triangle_offset;

	// e = x2 - x1, all derivatives are relative to x2
	double GetSpringEnergy(const Vector3d& e, double stiffness) const;
	Vector3d GetSpringEnergyGradient(const Vector3d& e, double stiffness) const;
	Matrix3d GetSpringEnergyHessian(const Vector3d& e, double stiffness) const;

};