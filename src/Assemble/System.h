//
// Created by hansljy on 10/7/22.
//

#ifndef FEM_SYSTEM_H
#define FEM_SYSTEM_H

#include "Object.hpp"
#include <string>
#include <map>
#include <vector>

class Constraint;
class SystemStepper;

class System final {
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
    int AddObject(Object* obj, const std::string& name);
    Object * GetObject(int idx);
    const Object* GetObject(int idx) const;
    void Initialize(const json &config);
    int GetOffset(int idx) const;
    int GetIndex(const std::string& name) const;

    ~System();
    System(const System& rhs) = delete;

    friend class Constraint;
	friend class SystemStepper;
    friend class SystemTarget;
    friend class DomainTarget;
	
	std::vector<Object*> _all_objs;
	std::vector<int> _level_bar;	

protected:
    int _dof;
    std::vector<Object*> _objs;
    std::vector<int> _offset;

    std::map<std::string, int> _index;

};

#endif //FEM_SYSTEM_H
