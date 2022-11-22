
//
// Created by hansljy on 11/10/22.
//

#ifndef FEM_OBJECTCOLLECTION_H
#define FEM_OBJECTCOLLECTION_H

#include "Object.h"
#include "Constraint/Constraint.h"
#include <memory>
#include <string>

class ObjectIterator;

class ObjectCollection {
public:
    virtual int AddObject(const Object& obj, const std::string& name) = 0;
    virtual const Object* GetObject(int idx) const = 0;
    virtual Object * GetObject(int idx) = 0;
    virtual void UpdateSettings(const json &config) = 0;

    virtual int AddConstraint(const Constraint& constraint) = 0;
    virtual int GetIndex(const std::string& name) const = 0;

    virtual int GetOffset(int idx) const = 0;

    virtual std::unique_ptr<ObjectIterator> GetIterator() = 0;
};

class ObjectIterator {
public:
    ObjectIterator(bool is_done) : _is_done(is_done) {}
    virtual void Forward() = 0;
    bool IsDone() const {return _is_done;};
    virtual Object* GetObject() = 0;
    virtual Matrix3d GetRotation() = 0;
    virtual Vector3d GetTranslation() = 0;

    virtual ~ObjectIterator() = default;

protected:
    bool _is_done;
};

#endif //FEM_OBJECTCOLLECTION_H
