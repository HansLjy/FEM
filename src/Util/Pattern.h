//
// Created by hansljy on 2022/2/22.
//

#ifndef FEM_PATTERN_H
#define FEM_PATTERN_H

#include <iostream>
#include <nlohmann/json.hpp>
using nlohmann::json;

#define DECLARE_GET_INSTANCE(classname) \
static classname* GetInstance();

#define DEFINE_GET_INSTANCE(classname) \
classname *classname::GetInstance() {  \
	static classname instance;         \
	return &instance;                  \
}

#define DECLARE_XXX_FACTORY(classname) \
class classname##Factory {             \
public:                                \
	static classname* Get##classname(const std::string& type, const json& config); \
};

#define BEGIN_DEFINE_XXX_FACTORY(classname) \
classname *classname##Factory::Get##classname(const std::string& type, const json& config) { \


#define ADD_PRODUCT(typename, classname) \
    if (type == typename) {              \
        return new classname(config);    \
    }

#define END_DEFINE_XXX_FACTORY  \
    return nullptr;             \
}

#define BASE_DECLARE_CLONE(classname) \
	virtual classname* Clone() const = 0;

#define MIDDLE_DECLARE_CLONE(classname) \
    classname* Clone() const override = 0;

#define DERIVED_DECLARE_CLONE(classname) \
	classname* Clone() const override;

#define DEFINE_CLONE(base, derived) \
base* derived::Clone() const {      \
    std::cerr << #derived << " cloned" << std::endl; \
	return new derived(*this);      \
}

#endif //FEM_PATTERN_H