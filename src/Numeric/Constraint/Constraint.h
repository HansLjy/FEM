//
// Created by hansljy on 10/8/22.
//

#ifndef FEM_CONSTRAINT_H
#define FEM_CONSTRAINT_H

#include "System.h"
#include "EigenAll.h"
#include <exception>

class Constraint {
public:

    virtual int GetSize() const = 0;

    //<- How many objects are involved in this constraint
    virtual int GetObjectsNum() const = 0;

    //<- Get the index of the objects involved in the constraint
    virtual int GetObjectIndex(int object_id) const = 0;

    //<- Set the corresponding offsets of the objects involved
    virtual void SetOffset(int offset_id, int offset) = 0;

    virtual VectorXd GetValue(const VectorXd &x) const = 0;
    virtual void GetGradient(const VectorXd &x, COO &coo, int x_offset) const = 0;

    virtual ~Constraint() = default;

    BASE_DECLARE_CLONE(Constraint)
};

class ConstraintFactory {
public:
    static Constraint *GetConstraint(const System &system, const nlohmann::json &config);
};

#endif //FEM_CONSTRAINT_H
