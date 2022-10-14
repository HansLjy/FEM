//
// Created by hansljy on 10/7/22.
//

#ifndef FEM_SYSTEM_H
#define FEM_SYSTEM_H

#include "Target.h"
#include "Object.h"
#include <string>
#include <map>
#include <vector>

class Constraint;
class Simulator;

class System final : public Target {
public:
    /**
     * @param size The initial size of the object array.
     *             Set default to 10
     */
    System(int size = 10);

    /**
     * Add object into the system
     * @param obj object to be added
     * @param name name of the object (for future access)
     * @return
     *  Upon success, return the index of the object
     *  otherwise, return -1.
     *  Failure could be the result of name collision
     */
    int AddObject(const Object& obj, const std::string& name);


    /**
     * Delete an object from the system by index
     * @param idx index of the object to be deleted
     * @return
     *  True if the operation is success, false otherwise
     *  Failure could be the result of out of range index
     */
    bool DeleteObject(int idx);

    const Object* GetObject(int idx) const;
    Object * GetObject(int idx);

    void UpdateSettings();

    /**
     * Delete an object from the system by name
     * @param name name of the object to be deleted
     * @return
     *  True if the operation is success, false otherwise
     *  Failure could be the result of non-exist name
     */
    bool DeleteObject(const std::string& name);

    VectorXd GetCoordinate() const override;
    VectorXd GetVelocity() const override;
    void SetCoordinate(const VectorXd& x);
    void SetVelocity(const VectorXd& v);

    void GetMass(SparseMatrixXd& mass) const override;

    double GetEnergy() const override;
    VectorXd GetEnergyGradient() const override;
    void GetEnergyHessian(SparseMatrixXd& hessian) const override;

    int AddConstraint(const Constraint& constraint);
    bool DeleteConstraint(int idx);

    VectorXd GetConstraint(const VectorXd &x) const override;

    // C(x + \Delta x) = C(X) + \nabla C(x) * \Delta x
    void GetConstraintGradient(SparseMatrixXd &gradient, const VectorXd &x) const override;

    int GetIndex(const std::string& name) const;

    friend class Constraint;
    friend class Simulator;

private:
    int _DOF;
    int _size;
    int _constraint_size;
    std::vector<Object*> _objs;
    std::vector<bool> _used;
    std::vector<int> _next_free;
    std::vector<int> _offset;
    int _first_free;

    std::vector<Constraint*> _constraints;

    std::map<std::string, int> _index;
};

#endif //FEM_SYSTEM_H
