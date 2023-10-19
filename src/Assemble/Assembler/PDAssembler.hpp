#pragma once

#include "EigenAll.h"
#include "Pattern.h"
#include "Object.hpp"

CONCEPT_MODEL_IDIOM_BEGIN(PDObject)
    ADD_INTERFACE_FUNCTION(int GetDOF() const, GetDOF())
    ADD_INTERFACE_FUNCTION(double GetEnergy(const Ref<const VectorXd>& x) const, GetEnergy(x))
    ADD_INTERFACE_FUNCTION(void GetGlobalMatrix(COO& coo, int x_offset, int y_offset) const, GetGlobalMatrix(coo, x_offset, y_offset))
    ADD_INTERFACE_FUNCTION(void LocalProject(const Ref<const VectorXd>& x, Ref<VectorXd> y) const, LocalProject(x, y))
CONCEPT_MODEL_IDIOM_CONCEPT
    ADD_CONCEPT_FUNCTION(int GetDOF() const)
    ADD_CONCEPT_FUNCTION(double GetEnergy(const Ref<const VectorXd>& x) const)
    ADD_CONCEPT_FUNCTION(void GetGlobalMatrix(COO& coo, int x_offset, int y_offset) const)
    ADD_CONCEPT_FUNCTION(void LocalProject(const Ref<const VectorXd>& x, Ref<VectorXd> y) const)
CONCEPT_MODEL_IDIOM_MODEL
    ADD_MODEL_FUNCTION(int GetDOF() const, GetDOF())
    ADD_MODEL_FUNCTION(double GetEnergy(const Ref<const VectorXd>& x) const, GetEnergy(x))
    ADD_MODEL_FUNCTION(void GetGlobalMatrix(COO& coo, int x_offset, int y_offset) const, GetGlobalMatrix(coo, x_offset, y_offset))
    ADD_MODEL_FUNCTION(void LocalProject(const Ref<const VectorXd>& x, Ref<VectorXd> y) const, LocalProject(x, y))
CONCEPT_MODEL_IDIOM_END

class PDAssembler {
public:
    double GetEnergy(const std::vector<PDObject>& objs, const Ref<const VectorXd>& x) const;
    void GetGlobalMatrix(const std::vector<PDObject>& objs, int total_dof, SparseMatrixXd& global) const;
    void GetGlobalMatrix(const std::vector<PDObject>& objs, COO& coo, int x_offset, int y_offset) const;
    void LocalProject(const std::vector<PDObject>& objs, const Ref<const VectorXd>& x, Ref<VectorXd> y) const;
};