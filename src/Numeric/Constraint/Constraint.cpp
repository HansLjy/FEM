//
// Created by hansljy on 10/14/22.
//

#include "Constraint.h"
#include "SampledObject/FixedPointConstraint.h"
#include "JsonUtil.h"

Constraint::Constraint(int num_objects, int size, const std::vector<int>& indices) : _num_objects(num_objects), _size(size), _index(indices) {
    _offsets.resize(num_objects);
}

int Constraint::GetSize() const {
    return _size;
}

int Constraint::GetObjectsNum() const {
    return _num_objects;
}

int Constraint::GetObjectIndex(int object_id) const {
    return _index[object_id];
}

void Constraint::SetOffset(int offset_id, int offset) {
    _offsets[offset_id] = offset;
}

Constraint *
ConstraintFactory::GetConstraint(const System &system, const nlohmann::json &config) {
    std::string type = config["type"];
    const auto& object_names = config["object-names"];
    std::vector<int> object_idx;
    for (auto& object_name : object_names) {
        object_idx.push_back(system.GetIndex(std::string(object_name)));
    }
    if (type == "fixed-point") {
        return new FixedPointConstraint(Json2Vec(config["fixed-point"]), object_idx[0], config["fixed-point-index"]);
    }
    return nullptr;
}