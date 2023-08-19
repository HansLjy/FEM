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
	explicit RenderShape(const json& config) : RenderShape(config["use-texture"], config["texture-path"]) {}
	RenderShape(bool use_texture, const std::string& texture_path = "") : _use_texture(use_texture) {
		if (use_texture) {
			_texture_path = std::string(TEXTURE_PATH) + "/" + texture_path;
		}
	}
	template<class Data> void Initialize(Data* data) {}

	// <- precompute uv coords
	template<class Data> bool IsUsingTexture(const Data* obj) const {return _use_texture;}
	template<class Data> const std::string& GetTexturePath(const Data* obj) const {return _texture_path;}
	template<class Data> void GetUVCoords(const Data* obj, MatrixXf& uv_coords) const {uv_coords = _uv_coords;}

protected:
	bool _use_texture = false;
	std::string _texture_path;
	Matrix<float, Dynamic, 2> _uv_coords;
};

class SampledRenderShape : public RenderShape {
public:
	explicit SampledRenderShape(const json& config) : RenderShape(config) {}
	SampledRenderShape() : RenderShape(false) {}
	SampledRenderShape(bool use_texture, const std::string& texture_path = "") : RenderShape(use_texture, texture_path) {}
	template<class Data> void GetRenderVertices(const Data *obj, MatrixXd &vertices) const;
	template<class Data> void GetRenderTopos(const Data *obj, MatrixXi &topos) const;
};

template<class ProxyRenderShape>
class ProxiedRenderShape {
public:
	ProxiedRenderShape(const json& config) : _proxy_render_shape(config) {}
	template<class Data> void Initialize(Data* data) {}
	template<class Data> void GetRenderVertices(const Data *data, MatrixXd &vertices) const;
	template<class Data> void GetRenderTopos(const Data *data, MatrixXi &topos) const;
	template<class Data> bool IsUsingTexture(const Data* data) const;
	template<class Data> const std::string& GetTexturePath(const Data* data) const;
	template<class Data> void GetUVCoords(const Data* data, MatrixXf& uv_coords) const;

protected:
	ProxyRenderShape _proxy_render_shape;
};

#include "FixedShape/FixedShape.hpp"
class FixedRenderShape : public RenderShape {
public:
	FixedRenderShape(const json& config)
		: RenderShape(config),
		  _vertices(FixedShapeFactory::Instance()->GetVertices(config["type"], config)),
		  _topos(FixedShapeFactory::Instance()->GetFaceTopo(config["type"], config)) {}

	template<class Data> void GetRenderVertices(const Data *obj, MatrixXd &vertices) const { vertices = _vertices; }

	template<class Data> void GetRenderTopos(const Data *obj, MatrixXi &topos) const { topos = _topos; }

protected:
    MatrixXd _vertices;
    MatrixXi _topos;
};

template<class Data>
void SampledRenderShape::GetRenderVertices(const Data *obj, MatrixXd &vertices) const {
	vertices = StackVector<double, 3>(obj->_x);
}

template<class Data>
void SampledRenderShape::GetRenderTopos(const Data *obj, MatrixXi &topos) const {
	topos = obj->_face_topo;
}

template<class ProxyRenderShape>
template<class Data> void ProxiedRenderShape<ProxyRenderShape>::GetRenderVertices(const Data *data, MatrixXd &vertices) const {
	_proxy_render_shape.GetRenderVertices(data->_proxy, vertices);
}

template<class ProxyRenderShape>
template<class Data> void ProxiedRenderShape<ProxyRenderShape>::GetRenderTopos(const Data *data, MatrixXi &topos) const {
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
