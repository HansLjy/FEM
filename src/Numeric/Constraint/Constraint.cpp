//
// Created by hansljy on 10/14/22.
//

#include "Constraint.h"
#include "SampledObject/FixedPointConstraint.h"
#include "SampledObject/JointConstraint.h"
#include "SampledObject/FixedAngleConstraint.h"
#include "JsonUtil.h"
#include "InertialSystem.h"

Constraint::Constraint(int num_objects, int size, const std::vector<int>& indices) : _num_objects(num_objects), _constraint_size(size), _object_index(indices) {
    _object_offsets.resize(num_objects);
}

int Constraint::GetSize() const {
    return _constraint_size;
}

int Constraint::GetObjectsNum() const {
    return _num_objects;
}

int Constraint::GetObjectIndex(int object_id) const {
    return _object_index[object_id];
}

void Constraint::SetOffset(int offset_id, int offset) {
    _object_offsets[offset_id] = offset;
}

Constraint *
ConstraintFactory::GetConstraint(const InertialSystem &system, const nlohmann::json &config) {
    std::string type = config["type"];
    const auto& object_names = config["object-names"];
    std::vector<int> object_idx;
    for (auto& object_name : object_names) {
        object_idx.push_back(system.GetIndex(std::string(object_name)));
    }
    if (type == "fixed-point") {
        return new FixedPointConstraint(Json2Vec(config["fixed-point"]), object_idx[0], config["fixed-point-index"]);
    }
    if (type == "joint") {
        return new JointConstraint(object_idx[0], config["joint-point-indices"][0], object_idx[1], config["joint-point-indices"][1]);
    }
    if (type == "fixed-angle") {
        return new FixedAngleConstraint(Json2Vec(config["direction"]), object_idx[0], config["point1"], config["point2"]);
    }
    return nullptr;
}