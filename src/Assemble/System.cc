//
// Created by hansljy on 10/8/22.
//

#include "System.h"
#include "Constraint/Constraint.h"
#include "spdlog/spdlog.h"

System::System(const nlohmann::json &config) : _dof(0) {
    const auto& objects_config = config["objects"];
    for (const auto& object_config : objects_config) {
        AddObject(ObjectFactory::GetObject(object_config["type"], object_config), object_config["name"]);
    }

    const auto& external_forces_config = config["external-forces"];
    for (const auto& external_force_config : external_forces_config) {
        int idx = GetIndex(external_force_config["object-name"]);
        GetObject(idx)->AddExternalForce(ExternalForceFactory::GetExternalForce(external_force_config["type"], external_force_config));
    }
}

int System::AddObject(Object* obj, const std::string &name) {
    int new_idx = _objs.size();
    auto result = _index.insert(std::pair<std::string, int>(name, new_idx));
    if (!result.second) {
        spdlog::error("Name {} already exist in the system!", name);
        return -1;
    }
    _objs.push_back(obj);
    _dof += obj->GetDOF();
    return new_idx;
}

Object * System::GetObject(int idx) {
    return _objs[idx];
}

const Object *System::GetObject(int idx) const {
    return _objs[idx];
}

int System::GetOffset(int idx) const {
    return _offset[idx];
}

int System::GetIndex(const std::string &name) const {
    const auto& result = _index.find(name);
    if (result == _index.end()) {
        spdlog::error("Name {} does not correspond to any objects in the system", name);
        return -1;
    }
    return (*result).second;
}

void System::Initialize(const json &config) {
	for (auto& obj : _objs) {
		obj->Initialize();
	}
	
    int cur_offset = 0;
    const int size = _objs.size();
    if (_offset.size() != size) {
        _offset.resize(size);
    }
    for (int i = 0; i < size; i++) {
        _offset[i] = cur_offset;
        cur_offset += _objs[i]->GetDOF();
    }
	_all_objs.clear();
	_level_bar.clear();
	int head = 0, tail = 0, level_end = 0;
	_level_bar.push_back(0);
	for (const auto& obj : _objs) {
		_all_objs.push_back(obj);
		tail++;
	}
	while (head < tail) {
		if (level_end == head) {
			_level_bar.push_back(level_end = tail);
		}
		const auto obj = _all_objs[head++];
		if (obj->IsDcomposed()) {
			const auto& decomposed_object = dynamic_cast<DecomposedObject*>(obj);
			for (const auto& child : decomposed_object->_children) {
				_all_objs.push_back(child);
				tail++;
			}
		}
	}

}

System::~System(){
    for (const auto& obj : _objs) {
        delete obj;
    }
}