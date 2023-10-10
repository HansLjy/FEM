#pragma once

#include "Pattern.h"
#include "EigenAll.h"

CONCEPT_MODEL_IDIOM_BEGIN(Renderable)
    ADD_INTERFACE_FUNCTION(bool IsUsingTexture() const, IsUsingTexture())
    ADD_INTERFACE_FUNCTION(const std::string& GetTexturePath() const, GetTexturePath())
    ADD_INTERFACE_FUNCTION(void GetUVCoords(MatrixXf& uv_coords) const, GetUVCoords(uv_coords))
    ADD_INTERFACE_FUNCTION(bool IsRenderTopoUpdated() const, IsRenderTopoUpdated())
    ADD_INTERFACE_FUNCTION(int GetRenderVertexNum() const, GetRenderVertexNum())
    ADD_INTERFACE_FUNCTION(void GetRenderVertices(Ref<MatrixXd> vertics) const, GetRenderVertices(vertics))
    ADD_INTERFACE_FUNCTION(int GetRenderFaceNum() const, GetRenderFaceNum())
    ADD_INTERFACE_FUNCTION(void GetRenderTopos(Ref<MatrixXi> topos) const, GetRenderTopos(topos))
CONCEPT_MODEL_IDIOM_CONCEPT
    ADD_CONCEPT_FUNCTION(bool IsUsingTexture() const)
    ADD_CONCEPT_FUNCTION(const std::string& GetTexturePath() const)
    ADD_CONCEPT_FUNCTION(void GetUVCoords(MatrixXf& uv_coords) const)
    ADD_CONCEPT_FUNCTION(bool IsRenderTopoUpdated() const)
    ADD_CONCEPT_FUNCTION(int GetRenderVertexNum() const)
    ADD_CONCEPT_FUNCTION(void GetRenderVertices(Ref<MatrixXd> vertices) const)
    ADD_CONCEPT_FUNCTION(int GetRenderFaceNum() const)
    ADD_CONCEPT_FUNCTION(void GetRenderTopos(Ref<MatrixXi> topos) const)
CONCEPT_MODEL_IDIOM_MODEL
    ADD_MODEL_FUNCTION(bool IsUsingTexture() const, IsUsingTexture())
    ADD_MODEL_FUNCTION(const std::string& GetTexturePath() const, GetTexturePath())
    ADD_MODEL_FUNCTION(void GetUVCoords(MatrixXf& uv_coords) const, GetUVCoords(uv_coords))
    ADD_MODEL_FUNCTION(bool IsRenderTopoUpdated() const, IsRenderTopoUpdated())
    ADD_MODEL_FUNCTION(int GetRenderVertexNum() const, GetRenderVertexNum())
    ADD_MODEL_FUNCTION(void GetRenderVertices(Ref<MatrixXd> vertics) const, GetRenderVertices(vertics))
    ADD_MODEL_FUNCTION(int GetRenderFaceNum() const, GetRenderFaceNum())
    ADD_MODEL_FUNCTION(void GetRenderTopos(Ref<MatrixXi> topos) const, GetRenderTopos(topos))
CONCEPT_MODEL_IDIOM_END