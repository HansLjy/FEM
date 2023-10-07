#include "GeneralSystem.hpp"

namespace {
	const bool gs_registered = Factory<System>::GetInstance()->Register("general", [](const json& config) {
		return new GeneralSystem(config);
	});
}

std::vector<Object *>& GeneralSystem::GetObjs() {
	return _objs;
}

GeneralSystem::GeneralSystem(const nlohmann::json &config) {
    const auto& objects_config = config["objects"];
    for (const auto& object_config : objects_config) {
        AddObject(
			Factory<Object>::GetInstance()->GetProduct(object_config["type"], object_config),
			object_config["name"]
		);
    }

    const auto& external_forces_config = config["external-forces"];
    for (const auto& external_force_config : external_forces_config) {
        int idx = GetIndex(external_force_config["object-name"]);
        _objs[idx]->AddExternalForce(external_force_config["type"], external_force_config);
    }
}

int GeneralSystem::AddObject(Object* obj, const std::string &name) {
    int new_idx = _objs.size();
    auto result = _index.insert(std::pair<std::string, int>(name, new_idx));
    if (!result.second) {
        spdlog::error("Name {} already exist in the system!", name);
        return -1;
    }
    _objs.push_back(obj);
    return new_idx;
}

int GeneralSystem::GetIndex(const std::string &name) const {
    const auto& result = _index.find(name);
    if (result == _index.end()) {
        spdlog::error("Name {} does not correspond to any objects in the system", name);
        return -1;
    }
    return (*result).second;
}

void GeneralSystem::Initialize(const json &config) {
	for (auto& obj : _objs) {
		obj->Initialize();
	}
}

GeneralSystem::~GeneralSystem(){
    for (const auto& obj : _objs) {
        delete obj;
    }
}