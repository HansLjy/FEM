//
// Created by hansljy on 10/13/22.
//

#ifndef FEM_RENDERSHAPE_H
#define FEM_RENDERSHAPE_H

#include "EigenAll.h"
#include "JsonUtil.h"
#include <functional>

/* Render Shape Policy */
class RenderShape {
public:
	RenderShape(const json& config) {
		if (config["use-texture"]) {
			_use_texture = true;
			_texture_path = std::string(TEXTURE_PATH) + "/" + std::string(config["texture-path"]);
		} else {
			_use_texture = false;
		}
	}

	RenderShape() = default;

	// <- precompute uv coords
	template<class Object> void PreCompute(const Object* obj);
	template<class Object> void GetSurface(const Object* obj, MatrixXd &vertices, MatrixXi &topos) const;
	template<class Object> bool IsUsingTexture(const Object* obj) const {return _use_texture;}
	template<class Object> const std::string& GetTexturePath(const Object* obj) const {return _texture_path;}
	template<class Object> void GetUVCoords(const Object* obj, MatrixXf& uv_coords) const {uv_coords = _uv_coords;}

protected:
	bool _use_texture = false;
	std::string _texture_path;
	Matrix<float, Dynamic, 2> _uv_coords;
};

class SampledRenderShape : public RenderShape {
public:
	template<class Object> void GetSurface(const Object *obj, MatrixXd &vertices, MatrixXi &topos) const;
};

class ProxyRenderShape {
public:
	template<class Object> void GetSurface(const Object *obj, MatrixXd &vertices, MatrixXi &topos) const {obj->_proxy->GetSurface(vertices, topos);}
	template<class Object> bool IsUsingTexture(const Object* obj) const {return obj->_proxy->IsUsingTexture();}
	template<class Object> const std::string& GetTexturePath(const Object* obj) const {return obj->_proxy->GetTexturePath();}
	template<class Object> void GetUVCoords(const Object* obj, MatrixXf& uv_coords) const {obj->_proxy->GetUVCoords(uv_coords);}
};

#include "FixedShape/FixedShape.hpp"
class FixedRenderShape : public RenderShape {
public:
	FixedRenderShape(const json& config)
		: RenderShape(config),
		  _vertices(FixedShapeFactory::Instance()->GetVertices(config["type"], config)),
		  _topos(FixedShapeFactory::Instance()->GetFaceTopo(config["type"], config)) {}

	template<class Object> void GetSurface(const Object *obj, MatrixXd &vertices, MatrixXi &topos) const {
        vertices = _vertices;
        topos = _topos;
    }

protected:
    MatrixXd _vertices;
    MatrixXi _topos;
};

template<class Object>
void SampledRenderShape::GetSurface(const Object *obj, MatrixXd &vertices, MatrixXi &topos) const {
	vertices = StackVector<double, 3>(obj->_x);
	topos = obj->_face_topo;
}

#define PROXY_RENDER_SHAPE(RenderShape) \
    void GetSurface(MatrixXd &vertices, MatrixXi &topos) const override {\
		return RenderShape::GetSurface(this, vertices, topos);\
	} \
    bool IsUsingTexture() const override { \
        return RenderShape::IsUsingTexture(this); \
    }\
	const std::string& GetTexturePath() const override { \
        return RenderShape::GetTexturePath(this);\
    }\
	void GetUVCoords(MatrixXf& uv_coords) const override {\
        RenderShape::GetUVCoords(this, uv_coords);\
	}\
	friend RenderShape;

#endif //FEM_RENDERSHAPE_H
