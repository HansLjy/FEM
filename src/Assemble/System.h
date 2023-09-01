//
// Created by hansljy on 10/7/22.
//

#ifndef FEM_SYSTEM_H
#define FEM_SYSTEM_H

#include "Object.hpp"
#include "JsonUtil.h"
#include <string>
#include <map>
#include <vector>

struct System final {
    explicit System(const json& config);
    void Initialize(const json &config);
    System(const System& rhs) = delete;
    ~System();

    std::vector<Object*> _objs;

protected:
	//<- id for object, -1 if failure
    int AddObject(Object* obj, const std::string& name);
    int GetIndex(const std::string& name) const;

    std::map<std::string, int> _index;
};

#endif //FEM_SYSTEM_H
