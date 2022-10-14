//
// Created by hansljy on 10/14/22.
//

#include "Constraint.h"
#include "Curve/FixedPoint.h"
#include "Curve/InextensibleCurve.h"
#include "JsonUtil.h"

Constraint *
ConstraintFactory::GetConstraint(const System &system, const nlohmann::json &config) {
    std::string type = config["type"];
    const auto& object_names = config["object-names"];
    std::vector<int> object_idx;
    for (auto& object_name : object_names) {
        object_idx.push_back(system.GetIndex(std::string(object_name)));
    }
    if (type == "curve-fixed-point") {
        const auto& curve = dynamic_cast<const Curve*>(system.GetObject(object_idx[0]));
        return new FixedPoint(*curve, object_idx[0], config["fixed-point-index"], Json2Vec(config["fixed-point"]));
    }
    if (type == "curve-inextensible") {
        const auto& curve = dynamic_cast<const Curve*>(system.GetObject(object_idx[0]));
        return new InextensibleCurve(*curve, object_idx[0]);
    }
    return nullptr;
}