#pragma once

#include "EigenAll.h"
#include "Pattern.h"
#include "Object.hpp"

CONCEPT_MODEL_IDIOM_BEGIN(PDObject)
    ADD_INTERFACE_FUNCTION(int GetDOF() const, GetDOF())
    ADD_INTERFACE_FUNCTION(void GetGlobalMatrix(COO& coo, int x_offset, int y_offset) const, GetGlobalMatrix(coo, x_offset, y_offset))
    ADD_INTERFACE_FUNCTION(void LocalProject(const Ref<const VectorXd>& x, Ref<VectorXd> y) const, LocalProject(x, y))
CONCEPT_MODEL_IDIOM_CONCEPT
    ADD_CONCEPT_FUNCTION(int GetDOF() const)
    ADD_CONCEPT_FUNCTION(void GetGlobalMatrix(COO& coo, int x_offset, int y_offset) const)
    ADD_CONCEPT_FUNCTION(void LocalProject(const Ref<const VectorXd>& x, Ref<VectorXd> y) const)
CONCEPT_MODEL_IDIOM_MODEL
    ADD_MODEL_FUNCTION(int GetDOF() const, GetDOF())
    ADD_MODEL_FUNCTION(void GetGlobalMatrix(COO& coo, int x_offset, int y_offset) const, GetGlobalMatrix(coo, x_offset, y_offset))
    ADD_MODEL_FUNCTION(void LocalProject(const Ref<const VectorXd>& x, Ref<VectorXd> y) const, LocalProject(x, y))
CONCEPT_MODEL_IDIOM_END

class PDAssembler : public InterfaceContainer<PDObject> {
public:
    void BindObjects(
        const typename std::vector<Object>::const_iterator &begin,
        const typename std::vector<Object>::const_iterator &end
    ) override;
    
    void GetGlobalMatrix(SparseMatrixXd& global) const;
    void GetGlobalMatrix(COO& coo, int x_offset, int y_offset) const;
    void LocalProject(const Ref<const VectorXd>& x, Ref<VectorXd> y, int offset) const;

protected:
    int _total_dof;
};