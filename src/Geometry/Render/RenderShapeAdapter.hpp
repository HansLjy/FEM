#pragma once

#include "EigenAll.h"
#include "JsonUtil.h"

template<class RenderShape, class Derived>
class RenderShapeAdapter : public RenderShape {
public:
    using RenderShape::RenderShape;
    RenderShapeAdapter(RenderShape&& rhs) : RenderShape(std::move(rhs)) {} 

    bool IsUsingTexture() const {
        return RenderShape::IsUsingTexture(static_cast<const Derived*>(this));
    }

	const std::string& GetTexturePath() const {
        return RenderShape::GetTexturePath(static_cast<const Derived*>(this));
    }

	void GetUVCoords(MatrixXf& uv_coords) const {
        RenderShape::GetUVCoords(static_cast<const Derived*>(this), uv_coords);
    }

	bool IsRenderTopoUpdated() const {
        return RenderShape::IsRenderTopoUpdated(static_cast<const Derived*>(this));
    }

    int GetRenderVertexNum() const {
        return RenderShape::GetRenderVertexNum(static_cast<const Derived*>(this));
    }

    void GetRenderVertices(Ref<MatrixXd> vertices) const {
        RenderShape::GetRenderVertices(static_cast<const Derived*>(this), vertices);
    }

    int GetRenderFaceNum() const {
        return RenderShape::GetRenderFaceNum(static_cast<const Derived*>(this));
    }

    void GetRenderTopos(Ref<MatrixXi> topos) const {
        RenderShape::GetRenderTopos(static_cast<const Derived*>(this), topos);
    }
};