#pragma once
#include <functional>
#include "JsonUtil.h"
#include "EigenAll.h"

class FixedShapeFactory {
public:
	using VerticesGenerator = std::function<MatrixXd(const json& config)>;
	using EdgeTopoGenerator = std::function<MatrixXi(const json& config)>;
	using FaceTopoGenerator = std::function<MatrixXi(const json& config)>;

	static FixedShapeFactory* Instance() {
		if (_the_factory == nullptr) {
			_the_factory = new FixedShapeFactory();
		}
		return _the_factory;
	}

	bool RegisterVerticesGenerator(const std::string& name, const VerticesGenerator& generator) {
		return _vertices_generator.insert(std::make_pair(name, generator)).second;
	}

	bool RegisterEdgeTopoGenerator(const std::string& name, const EdgeTopoGenerator& generator) {
		return _edge_topo_generator.insert(std::make_pair(name, generator)).second;
	}

	bool RegisterFaceTopoGenerator(const std::string& name, const FaceTopoGenerator& generator) {
		return _face_topo_generator.insert(std::make_pair(name, generator)).second;
	}

	MatrixXd GetVertices(const std::string& name, const json& config) {
		return _vertices_generator[name](config);
	}

	MatrixXi GetEdgeTopo(const std::string& name, const json& config) {
		return _edge_topo_generator[name](config);
	}

	MatrixXi GetFaceTopo(const std::string& name, const json& config) {
		return _face_topo_generator[name](config);
	}

private:
	std::map<std::string, VerticesGenerator> _vertices_generator;
	std::map<std::string, EdgeTopoGenerator> _edge_topo_generator;
	std::map<std::string, FaceTopoGenerator> _face_topo_generator;

	FixedShapeFactory() = default;
	~FixedShapeFactory() = default;

	static FixedShapeFactory* _the_factory;
};
