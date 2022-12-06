//
// Created by hansljy on 10/7/22.
//

#ifndef FEM_SYSTEM_H
#define FEM_SYSTEM_H

#include "Object.h"
#include "ObjectIterator.h"
#include "Target.h"
#include <string>
#include <map>
#include <vector>

class Constraint;
class SystemIterator;

class System {
public:
    explicit System(const json& config);

    int GetDOF() {return _dof;}

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
    Object * GetObject(int idx);
    const Object* GetObject(int idx) const;
    virtual void UpdateSettings(const json &config);
    int GetOffset(int idx) const;
    int AddConstraint(const Constraint& constraint);
    int GetIndex(const std::string& name) const;

    virtual std::unique_ptr<ObjectIterator> GetIterator();

    virtual ~System();
    System(const System& rhs) = delete;

    friend class Constraint;
    friend class SystemIterator;
    friend class SystemTarget;
    friend class DomainTarget;

protected:
    int _dof;
    int _constraint_size;
    std::vector<Object*> _objs;
    std::vector<int> _offset;

    std::vector<Constraint*> _constraints;
    std::map<std::string, int> _index;
};

#include "ObjectIterator.h"

class SystemIterator : public ObjectIterator {
public:
    explicit SystemIterator(System& system);

    void Forward() override;
    Object * GetObject() override;

    std::shared_ptr<ObjectIterator> Clone() const override {
        return std::shared_ptr<ObjectIterator> (new SystemIterator(*this));
    }

private:
    System* _system;
    int _obj_id;
    int _cur_size;
};

DECLARE_XXX_FACTORY(System)


#endif //FEM_SYSTEM_H
