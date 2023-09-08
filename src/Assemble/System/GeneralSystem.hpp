//
// Created by hansljy on 10/7/22.
//

#ifndef FEM_SYSTEM_H
#define FEM_SYSTEM_H

#include "System.hpp"
#include <string>
#include <map>
#include <vector>

struct GeneralSystem : public System {
    explicit GeneralSystem(const json& config);
	~GeneralSystem();
	
    void Initialize(const json &config);
	std::vector<Object *> & GetObjs() override;

protected:
	//<- id for object, -1 if failure
    int AddObject(Object* obj, const std::string& name);
    int GetIndex(const std::string& name) const;

    std::map<std::string, int> _index;
    std::vector<Object*> _objs;
};

#endif //FEM_SYSTEM_H
