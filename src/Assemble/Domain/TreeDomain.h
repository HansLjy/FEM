//
// Created by hansljy on 11/9/22.
//

#ifndef FEM_TREEDOMAIN_H
#define FEM_TREEDOMAIN_H

#include "Domain.h"

class ReducedTreeTrunk;

class TreeDomain : public Domain {
public:
    explicit TreeDomain(const json& config);

    void UpdateSettings(const nlohmann::json &config) override;
    void CalculateSubdomainFrame(const Eigen::VectorXd &a) override;
    void SetObjectExtraForce() override;

    SparseMatrixXd GetSubdomainProjection(const nlohmann::json &position) override;
    void RecordSubdomain(const nlohmann::json &position) override;

//    DERIVED_DECLARE_CLONE(Domain)
protected:
    ReducedTreeTrunk* _tree_trunk;
    int _tree_trunk_offset;
    std::vector<double> _positions;
};

#endif //FEM_TREEDOMAIN_H
