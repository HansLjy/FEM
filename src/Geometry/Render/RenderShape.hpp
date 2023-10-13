//
// Created by hansljy on 10/13/22.
//

#pragma once

#include "EigenAll.h"
#include "JsonUtil.h"
#include <functional>

/* Render Shape Policy */
class RenderShape {
public:

	RenderShape(const RenderShape& rhs) = delete;
	RenderShape(RenderShape&& rhs) = default;
	template<class Data> void Initialize(Data* data) {}

	// <- precompute uv coords
	template<class Data> bool IsUsingTexture(const Data* obj) const {return _use_texture;}
	template<class Data> const std::string& GetTexturePath(const Data* obj) const {return _texture_path;}
	template<class Data> void GetUVCoords(const Data* obj, MatrixXf& uv_coords) const {uv_coords = _uv_coords;}

	template<class Data> bool HasOuterFrame(const Data* obj) const {return false;}
	template<class Data> void GetFrameVertices(const Data* obj, Ref<MatrixXd> vertices) const {}
	template<class Data> void GetFrameTopo(const Data* obj, Ref<MatrixXi> topo) const {}

	template<class Data> bool IsRenderTopoUpdated(Data* data) const {return false;}
	template<class Data> bool IsBBTopoUpdated(Data* data) const {return false;}


protected:
	bool _have_bounding_box = false;
	bool _use_texture = false;
	std::string _texture_path;
	Matrix<float, Dynamic, 2> _uv_coords;
	
	RenderShape(bool have_bounding_box, bool use_texture, const std::string& texture_path)
	: _have_bounding_box(have_bounding_box), _use_texture(use_texture) {
		if (use_texture) {
			_texture_path = std::string(TEXTURE_PATH) + "/" + texture_path;
		}
	}
};

class SampledRenderShape : public RenderShape {
public:
	SampledRenderShape(bool have_bounding_box, bool use_texture, const std::string& texture_path) :
		RenderShape(have_bounding_box, use_texture, texture_path) {}

	SampledRenderShape(const SampledRenderShape& rhs) = delete;
	SampledRenderShape(SampledRenderShape&& rhs) = default;

	static inline SampledRenderShape CreateFromConfig(const json& config);

	template<class Data> int GetRenderVertexNum(const Data *obj) const;
	template<class Data> void GetRenderVertices(const Data *obj, Ref<MatrixXd> vertices) const;
	template<class Data> int GetRenderFaceNum(const Data* obj) const;
	template<class Data> void GetRenderTopos(const Data *obj, Ref<MatrixXi> topos) const;
};

template<class ProxyRenderShape>
class ProxiedRenderShape {
public:
	template<class Data> void Initialize(Data* data) {}
	template<class Data> void GetRenderVertices(const Data *data, Ref<MatrixXd> vertices) const;
	template<class Data> void GetRenderTopos(const Data *data, Ref<MatrixXi> topos) const;
	template<class Data> bool IsUsingTexture(const Data* data) const;
	template<class Data> const std::string& GetTexturePath(const Data* data) const;
	template<class Data> void GetUVCoords(const Data* data, MatrixXf& uv_coords) const;

	template<class Data> bool HasOuterFrame(const Data* obj) const;
	template<class Data> void GetFrameVertices(const Data* obj, Ref<MatrixXd> vertices) const;
	template<class Data> void GetFrameTopo(const Data* obj, Ref<MatrixXi> topo) const;

	template<class Data> bool IsTopoUpdated(Data* data) const;
	template<class Data> bool IsBBTopoUpdated(Data* data) const;
protected:
	ProxyRenderShape _proxy_render_shape;
};

#include "FixedShape/FixedShape.hpp"
class FixedRenderShape : public RenderShape {
public:
	FixedRenderShape(const MatrixXd& vertices, const MatrixXi& topo) :
		RenderShape(false, false, ""), _vertices(vertices), _topo(topo) {}
	
	FixedRenderShape(const FixedRenderShape& rhs) = delete;
	FixedRenderShape(FixedRenderShape&& rhs) = default;
	
	template<class Data> void GetRenderVertices(const Data *obj, MatrixXd &vertices) const { vertices = _vertices; }
	template<class Data> void GetRenderTopos(const Data *obj, MatrixXi &topo) const { topo = _topo; }

protected:
    MatrixXd _vertices;
    MatrixXi _topo;
};


template<class Data>
int SampledRenderShape::GetRenderVertexNum(const Data *obj) const {
	return obj->_num_points;
}

template<class Data>
int SampledRenderShape::GetRenderFaceNum(const Data* obj) const {
	return obj->_num_faces;
}

template<class Data>
void SampledRenderShape::GetRenderVertices(const Data *obj, Ref<MatrixXd> vertices) const {
	vertices = StackVector<double, 3>(obj->_x.topRows(obj->_dof));
}

template<class Data>
void SampledRenderShape::GetRenderTopos(const Data *obj, Ref<MatrixXi> topos) const {
	topos = obj->_face_topo.topRows(obj->_num_faces);
}

SampledRenderShape SampledRenderShape::CreateFromConfig(const json& config) {
	return {
		config["have-bounding-box"],
		config["use-texture"],
		config["texture-path"]
	};
}


template<class ProxyRenderShape>
template<class Data> void ProxiedRenderShape<ProxyRenderShape>::GetRenderVertices(const Data *data, Ref<MatrixXd> vertices) const {
	_proxy_render_shape.GetRenderVertices(data->_proxy, vertices);
}

template<class ProxyRenderShape>
template<class Data> void ProxiedRenderShape<ProxyRenderShape>::GetRenderTopos(const Data *data, Ref<MatrixXi> topos) const {
	_proxy_render_shape.GetRenderTopos(data->_proxy, topos);
}

template<class ProxyRenderShape>
template<class Data> bool ProxiedRenderShape<ProxyRenderShape>::IsUsingTexture(const Data* data) const {
	return _proxy_render_shape.IsUsingTexture(data->_proxy);
}

template<class ProxyRenderShape>
template<class Data> const std::string& ProxiedRenderShape<ProxyRenderShape>::GetTexturePath(const Data* data) const {
	return _proxy_render_shape.GetTexturePath(data->_proxy);
}

template<class ProxyRenderShape>
template<class Data> void ProxiedRenderShape<ProxyRenderShape>::GetUVCoords(const Data* data, MatrixXf& uv_coords) const {
	_proxy_render_shape.GetUVCoords(data->_proxy, uv_coords);
}


template<class ProxyRenderShape>
template<class Data> bool ProxiedRenderShape<ProxyRenderShape>::HasOuterFrame(const Data* obj) const {
	return _proxy_render_shape.HasOuterFrame(obj->_proxy);
}

template<class ProxyRenderShape>
template<class Data> void ProxiedRenderShape<ProxyRenderShape>::GetFrameVertices(const Data* obj, Ref<MatrixXd> vertices) const {
	_proxy_render_shape.GetFrameVertices(obj->_proxy, vertices);
}

template<class ProxyRenderShape>
template<class Data> void ProxiedRenderShape<ProxyRenderShape>::GetFrameTopo(const Data* obj, Ref<MatrixXi> topo) const {
	_proxy_render_shape.GetFrameTopo(obj->_proxy, topo);
}

template<class ProxyRenderShape>
template<class Data> bool ProxiedRenderShape<ProxyRenderShape>::IsTopoUpdated(Data* data) const {
	return _proxy_render_shape.IsTopoUpdated(data->_proxy);
}

template<class ProxyRenderShape>
template<class Data> bool ProxiedRenderShape<ProxyRenderShape>::IsBBTopoUpdated(Data* data) const {
	return _proxy_render_shape.IsBBTopoUpdated(data->_proxy);
}