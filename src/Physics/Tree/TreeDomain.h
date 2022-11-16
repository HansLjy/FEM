//
// Created by hansljy on 11/9/22.
//

#ifndef FEM_TREEDOMAIN_H
#define FEM_TREEDOMAIN_H

#include "Domain.h"

class TreeDomain : public Domain {
public:
    explicit TreeDomain(const json& config) : Domain(config), _tree_trunk_id(_subdomains.empty() ? -1 : _system.GetIndex("trunk")) {}

    void CalculateSubdomainFrame(const Eigen::VectorXd &a) override;
    SparseMatrixXd GetSubdomainProjection(const nlohmann::json &position) override;
    void RecordSubdomain(const nlohmann::json &position) override;

//    DERIVED_DECLARE_CLONE(Domain)
protected:
    int _tree_trunk_id;
    std::vector<double> _positions;
};

#endif //FEM_TREEDOMAIN_H
