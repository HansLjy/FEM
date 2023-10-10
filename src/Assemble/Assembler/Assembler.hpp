//
// Created by hansljy on 10/11/22.
//

#pragma once

#include "EigenAll.h"
#include "Pattern.h"
#include "Object.hpp"

CONCEPT_MODEL_IDIOM_BEGIN(Coordinated)
	ADD_INTERFACE_FUNCTION(int GetDOF() const, GetDOF())
	ADD_INTERFACE_FUNCTION(void GetCoordinate(Ref<VectorXd> x) const, GetCoordinate(x))
	ADD_INTERFACE_FUNCTION(void GetVelocity(Ref<VectorXd> v) const, GetVelocity(v))
	ADD_INTERFACE_FUNCTION(void SetCoordinate(const Ref<const VectorXd> &x) const, SetCoordinate(x))
	ADD_INTERFACE_FUNCTION(void SetVelocity(const Ref<const VectorXd> &v) const, SetVelocity(v))
CONCEPT_MODEL_IDIOM_CONCEPT
	ADD_CONCEPT_FUNCTION(int GetDOF() const)
	ADD_CONCEPT_FUNCTION(void GetCoordinate(Ref<VectorXd> x) const)
	ADD_CONCEPT_FUNCTION(void GetVelocity(Ref<VectorXd> v) const)
	ADD_CONCEPT_FUNCTION(void SetCoordinate(const Ref<const VectorXd> &x) const)
	ADD_CONCEPT_FUNCTION(void SetVelocity(const Ref<const VectorXd> &v) const)
CONCEPT_MODEL_IDIOM_MODEL
	ADD_MODEL_FUNCTION(int GetDOF() const, GetDOF())
	ADD_MODEL_FUNCTION(void GetCoordinate(Ref<VectorXd> x) const, GetCoordinate(x))
	ADD_MODEL_FUNCTION(void GetVelocity(Ref<VectorXd> v) const, GetVelocity(v))
	ADD_MODEL_FUNCTION(void SetCoordinate(const Ref<const VectorXd> &x) const, SetCoordinate(x))
	ADD_MODEL_FUNCTION(void SetVelocity(const Ref<const VectorXd> &v) const, SetVelocity(v))
CONCEPT_MODEL_IDIOM_END

CONCEPT_MODEL_IDIOM_BEGIN(Massed)
	ADD_INTERFACE_FUNCTION(int GetDOF() const, GetDOF())
	ADD_INTERFACE_FUNCTION(void GetMass(COO& coo, int offset_x, int offset_y) const, GetMass(coo, offset_x, offset_y))
CONCEPT_MODEL_IDIOM_CONCEPT
	ADD_CONCEPT_FUNCTION(int GetDOF() const)
	ADD_CONCEPT_FUNCTION(void GetMass(COO& coo, int offset_x, int offset_y) const)
CONCEPT_MODEL_IDIOM_MODEL
	ADD_MODEL_FUNCTION(int GetDOF() const, GetDOF())
	ADD_MODEL_FUNCTION(void GetMass(COO& coo, int offset_x, int offset_y) const, GetMass(coo, offset_x, offset_y))
CONCEPT_MODEL_IDIOM_END

CONCEPT_MODEL_IDIOM_BEGIN(Energied)
	ADD_INTERFACE_FUNCTION(int GetDOF() const, GetDOF())
	ADD_INTERFACE_FUNCTION(double GetPotential(const Ref<const VectorXd>& x) const, GetPotential(x))
	ADD_INTERFACE_FUNCTION(VectorXd GetPotentialGradient(const Ref<const VectorXd> &x) const, GetPotentialGradient(x))
	ADD_INTERFACE_FUNCTION(
		void GetPotentialHessian(const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const,
		GetPotentialHessian(x, coo, x_offset, y_offset)
	)
CONCEPT_MODEL_IDIOM_CONCEPT
	ADD_CONCEPT_FUNCTION(int GetDOF() const)
	ADD_CONCEPT_FUNCTION(double GetPotential(const Ref<const VectorXd>& x) const)
	ADD_CONCEPT_FUNCTION(VectorXd GetPotentialGradient(const Ref<const VectorXd> &x) const)
	ADD_CONCEPT_FUNCTION(void GetPotentialHessian(const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const)
CONCEPT_MODEL_IDIOM_MODEL
	ADD_MODEL_FUNCTION(int GetDOF() const, GetDOF())
	ADD_MODEL_FUNCTION(double GetPotential(const Ref<const VectorXd>& x) const, GetPotential(x))
	ADD_MODEL_FUNCTION(VectorXd GetPotentialGradient(const Ref<const VectorXd> &x) const, GetPotentialGradient(x))
	ADD_MODEL_FUNCTION(
		void GetPotentialHessian(const Ref<const VectorXd> &x, COO &coo, int x_offset, int y_offset) const,
		GetPotentialHessian(x, coo, x_offset, y_offset)
	)
CONCEPT_MODEL_IDIOM_END

CONCEPT_MODEL_IDIOM_BEGIN(ExternalForced)
	ADD_INTERFACE_FUNCTION(int GetDOF() const, GetDOF())
	ADD_INTERFACE_FUNCTION(VectorXd GetExternalForce() const, GetExternalForce())
CONCEPT_MODEL_IDIOM_CONCEPT
	ADD_CONCEPT_FUNCTION(int GetDOF() const)
	ADD_CONCEPT_FUNCTION(VectorXd GetExternalForce() const)
CONCEPT_MODEL_IDIOM_MODEL
	ADD_MODEL_FUNCTION(VectorXd GetExternalForce() const, GetExternalForce())
	ADD_MODEL_FUNCTION(int GetDOF() const, GetDOF())
CONCEPT_MODEL_IDIOM_END

// virtual void GetExternalForce(Ref<VectorXd> force) const = 0;

class CoordinateAssembler : public InterfaceContainer<Coordinated> {
public:
	void BindObjects(
		const typename std::vector<Object>::const_iterator &begin,
		const typename std::vector<Object>::const_iterator &end
	) override;

    int GetDOF() const;

    void GetCoordinate(Ref<VectorXd> x) const;
    void GetVelocity(Ref<VectorXd> v) const;

    void SetCoordinate(const Ref<const VectorXd> &x) const;
    void SetVelocity(const Ref<const VectorXd> &v) const;

protected:
	int _total_dof;
};

class MassAssembler : public InterfaceContainer<Massed> {
public:
	void BindObjects(
		const typename std::vector<Object>::const_iterator& begin,
		const typename std::vector<Object>::const_iterator& end
	) override;
	void GetMass(SparseMatrixXd& mass) const;
	void GetMass(COO& coo, int offset_x, int offset_y) const;

protected:
	int _total_dof;
};

class EnergyAssembler : public InterfaceContainer<Energied> {
public:
	void BindObjects(
		const typename std::vector<Object>::const_iterator &begin,
		const typename std::vector<Object>::const_iterator &end
	) override;
	double GetPotentialEnergy(const Ref<const VectorXd>& x) const;
	void GetPotentialGradient(const Ref<const VectorXd> &x, Ref<VectorXd> gradient) const;
	void GetPotentialHessian(const Ref<const VectorXd>& x, SparseMatrixXd& hessian) const;
	void GetPotentialHessian(const Ref<const Eigen::VectorXd> &x, COO &coo, int offset_x, int offset_y) const;

protected:
	int _total_dof;
};

class ExternalForceAssembler : public InterfaceContainer<ExternalForced> {
public:
	void GetExternalForce(Ref<VectorXd> force) const;
};