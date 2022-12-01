//
// Created by hansljy on 10/8/22.
//

#ifndef FEM_CONSTRAINT_H
#define FEM_CONSTRAINT_H

#include "EigenAll.h"
#include "Pattern.h"
#include <exception>

class System;

class Constraint {
public:

    Constraint(int num_objects, int size, const std::vector<int>& indices);

    int GetSize() const;

    //<- How many objects are involved in this constraint
    int GetObjectsNum() const;

    //<- Get the index of the objects involved in the constraint
    int GetObjectIndex(int object_id) const;

    //<- Set the corresponding offsets of the objects involved
    void SetOffset(int offset_id, int offset);

    virtual VectorXd GetValue(const VectorXd &x) const = 0;
    virtual void GetGradient(const VectorXd &x, COO &coo, int x_offset) const = 0;

    virtual ~Constraint() = default;

    BASE_DECLARE_CLONE(Constraint)

protected:
    const int _num_objects;
    const int _constraint_size;
    std::vector<int> _object_index;
    std::vector<int> _object_offsets;
};

class ConstraintFactory {
public:
    static Constraint *GetConstraint(const System &system, const nlohmann::json &config);
};

#endif //FEM_CONSTRAINT_H
