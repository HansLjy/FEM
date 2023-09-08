//
// Created by hansljy on 10/11/22.
//

#pragma once

#include "EigenAll.h"
#include "Pattern.h"
#include "System/System.hpp"

// Assemble the bunch of objects
class Assembler {
public:
	virtual void BindSystem(System& system);

	virtual void BindObjects (
		const typename std::vector<Object*>::const_iterator& begin,
		const typename std::vector<Object*>::const_iterator& end
	);
	virtual void BindObjects(const std::vector<Object*>& objs);

    virtual int GetDOF() const = 0;

    virtual void GetCoordinate(Ref<VectorXd> x) const = 0;
    virtual void GetVelocity(Ref<VectorXd> v) const = 0;

    virtual void SetCoordinate(const Ref<const VectorXd> &x) = 0;
    virtual void SetVelocity(const Ref<const VectorXd> &v) = 0;

    virtual void GetMass(SparseMatrixXd& mass) const = 0;
    virtual void GetMass(COO& coo, int offset_x, int offset_y) const = 0;

    virtual double GetPotentialEnergy(const Ref<const VectorXd>& x) const = 0;
    virtual void GetPotentialEnergyGradient(const Ref<const VectorXd> &x, Ref<VectorXd> gradient) const = 0;
	virtual void GetPotentialEnergyHessian(const Ref<const VectorXd>& x, SparseMatrixXd& hessian) const = 0;
    virtual void GetPotentialEnergyHessian(const Ref<const Eigen::VectorXd> &x, COO &coo, int offset_x, int offset_y) const = 0;

    virtual void GetExternalForce(Ref<VectorXd> force) const = 0;

    virtual ~Assembler() = default;

protected:
	std::vector<Object*> _objs;
};
