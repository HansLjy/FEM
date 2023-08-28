#pragma once

#include "RenderShape.hpp"

class GridBasedShape : public ProxiedRenderShape<SampledRenderShape> {
public:
	template<class Data> bool HasOuterFrame(const Data* obj) const {
        return true;
    }
	
    template<class Data> void GetFrameVertices(const Data* obj, MatrixXd& vertices) const {
        vertices = StackVector<double, 3>(obj->_x);
    }

	template<class Data> void GetFrameTopo(const Data* obj, MatrixXi& topo) const {
        topo = obj->_edge_topo;
    }
};

