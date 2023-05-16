#pragma once

#include "RenderShape.hpp"

class LeafRenderShape : public SampledRenderShape {
public:
    LeafRenderShape() : SampledRenderShape(true, "leaf.jpeg") {}
	template<class Object> void PreCompute(const Object* obj);
};

template<class Object>
void LeafRenderShape::PreCompute(const Object* obj) {
    const VectorXd& uv_coords = obj->_uv_coord;
    int num_points = uv_coords.size() / 2;
    _uv_coords.resize(num_points, 2);
    for (int i = 0, i2 = 0; i < num_points; i++, i2 += 2) {
        _uv_coords.row(i) = uv_coords.segment<2>(i2).transpose().cast<float>();
    }
}
