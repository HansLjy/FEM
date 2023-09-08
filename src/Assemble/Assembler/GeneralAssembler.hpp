#pragma once

#include "EigenAll.h"
#include "Pattern.h"
#include "Assembler.hpp"
#include "Object.hpp"

class GeneralAssembler : public Assembler {
public:
	explicit GeneralAssembler(const json& config);

	void BindSystem(System& system) override;
	void BindObjects(const typename std::vector<Object *>::const_iterator &begin, const typename std::vector<Object *>::const_iterator &end) override;
	void BindObjects(const std::vector<Object *> &objs) override;

    int GetDOF() const override {
        return _dof;
    }

    void GetCoordinate(Ref<VectorXd> x) const override;
    void GetVelocity(Ref<VectorXd> v) const override;

    void SetCoordinate(const Ref<const VectorXd> &x) override;
    void SetVelocity(const Ref<const VectorXd> &v) override;

    void GetMass(SparseMatrixXd& mass) const override {
        COO coo;
        GetMass(coo, 0, 0);
        mass.resize(GetDOF(), GetDOF());
        mass.setFromTriplets(coo.begin(), coo.end());
    }
    void GetMass(COO& coo, int offset_x, int offset_y) const override;

    virtual double GetPotentialEnergy(const Ref<const VectorXd>& x) const override;
    virtual void GetPotentialEnergyGradient(const Ref<const VectorXd> &x, Ref<VectorXd> gradient) const override;
    void GetPotentialEnergyHessian(const Ref<const VectorXd>& x, SparseMatrixXd& hessian) const override {
        COO coo;
        GetPotentialEnergyHessian(x, coo, 0, 0);
        hessian.resize(GetDOF(), GetDOF());
        hessian.setFromTriplets(coo.begin(), coo.end());
    }
    virtual void GetPotentialEnergyHessian(const Ref<const Eigen::VectorXd> &x, COO &coo, int offset_x, int offset_y) const override;
    void GetExternalForce(Ref<VectorXd> force) const override;

    virtual ~GeneralAssembler() = default;

protected:
    int _dof;
    std::vector<int> _offsets;
};


