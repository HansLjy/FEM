#include "Object.hpp"

Deleter* Deleter::_the_deletor = nullptr;
Creator* Creator::_the_creator = nullptr;

Object::Object(const std::string& type, void* ptr)
    : _type(type), _ptr({ptr, [type](void* ptr) {
        Deleter::GetInstance()->Delete(type, ptr);
    }}) {}