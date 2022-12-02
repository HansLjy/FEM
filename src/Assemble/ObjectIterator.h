
//
// Created by hansljy on 11/10/22.
//

#ifndef FEM_OBJECTCOLLECTION_H
#define FEM_OBJECTCOLLECTION_H

#include "Object.h"

class ObjectIterator {
public:
    ObjectIterator(bool is_done) : _is_done(is_done) {}
    virtual void Forward() = 0;
    bool IsDone() const {return _is_done;};
    virtual Object* GetObject() = 0;
    virtual Matrix3d GetRotation() = 0;
    virtual Vector3d GetTranslation() = 0;

    virtual std::shared_ptr<ObjectIterator> Clone() const = 0;

    virtual ~ObjectIterator() = default;

protected:
    bool _is_done;
};

#endif //FEM_OBJECTCOLLECTION_H
