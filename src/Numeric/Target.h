//
// Created by hansljy on 10/11/22.
//

#ifndef FEM_TARGET_H
#define FEM_TARGET_H

#include "EigenAll.h"
#include "Pattern.h"

class Object;
class ObjectIterator;

/**
 * Target is the general target problem encountered
 * in the field of physics-based simulation
 */
class Target {
public:
	explicit Target(const std::vector<Object*>& objs, int begin, int end, const json& config);

    int GetDOF() const {
        return _dof;
    }

    void GetCoordinate(Ref<VectorXd> x) const;
    void GetVelocity(Ref<VectorXd> v) const;

    void SetCoordinate(const Ref<const VectorXd> &x);
    void SetVelocity(const Ref<const VectorXd> &v);

    void GetMass(SparseMatrixXd& mass) const {
        COO coo;
        GetMass(coo, 0, 0);
        mass.resize(GetDOF(), GetDOF());
        mass.setFromTriplets(coo.begin(), coo.end());
    }
    void GetMass(COO& coo, int offset_x, int offset_y) const;

    virtual double GetPotentialEnergy(const Ref<const VectorXd>& x) const;
    virtual void GetPotentialEnergyGradient(const Ref<const VectorXd> &x, Ref<VectorXd> gradient) const;
    void GetPotentialEnergyHessian(const Ref<const VectorXd>& x, SparseMatrixXd& hessian) const {
        COO coo;
        GetPotentialEnergyHessian(x, coo, 0, 0);
        hessian.resize(GetDOF(), GetDOF());
        hessian.setFromTriplets(coo.begin(), coo.end());
    }
    virtual void GetPotentialEnergyHessian(const Ref<const Eigen::VectorXd> &x, COO &coo, int offset_x, int offset_y) const;
    void GetExternalForce(Ref<VectorXd> force) const;

    virtual ~Target() = default;

protected:
    int _dof;
    std::vector<Object*> _objs;
    std::vector<int> _offsets;
};

class TargetFactory {
public:
    static Target* GetTarget(const std::vector<Object*> objs, int begin, int end, const std::string& type, const json& config);
};

#endif //FEM_TARGET_H
