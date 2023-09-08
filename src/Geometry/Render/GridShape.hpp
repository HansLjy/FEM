#pragma once

#include <iostream>
#include "RenderShape.hpp"

class GridShape : public RenderShape {
public:
	explicit GridShape(const json& config) : RenderShape(config) {}
	GridShape(bool have_bounding_box) : RenderShape(have_bounding_box, false) {}

	template<class Data> void GetRenderVertices(const Data *obj, MatrixXd &vertices) const {
		vertices = StackVector<double, 3>(obj->_proxy->_x.head(obj->_proxy->_dof));
	}
	
	template<class Data> void GetRenderTopos(const Data *obj, MatrixXi &topos) const {
		topos = obj->_proxy->_face_topo.topRows(obj->_proxy->_num_faces);
	}

	template<class Data> bool HasOuterFrame(const Data* obj) const {
        return true;
    }
	
    template<class Data> void GetFrameVertices(const Data* obj, MatrixXd& vertices) const {
        vertices = StackVector<double, 3>(obj->_x);
    }

	template<class Data> void GetFrameTopo(const Data* obj, MatrixXi& topo) const {
        topo = obj->_face_topo;
    }
	
	template<class Data> bool IsTopoUpdated(Data* data) const {
		bool result = data->_topo_changed;
		data->_topo_changed = false;
		return result;
	}

	template<class Data> bool IsBBTopoUpdated(Data* data) const {
		bool result = data->_bb_topo_changed;
		data->_bb_topo_changed = false;
		return result;
	}
};

