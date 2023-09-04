#pragma once
#include "JsonUtil.h"
#include "EigenAll.h"
#include "Objects/Grid.hpp"
#include "Objects/Triangle.hpp"

struct GeometryReconstructSystem {
	explicit GeometryReconstructSystem(const json& config);
	GeometryReconstructSystem(const GeometryReconstructSystem& rhs) = delete;

protected:
	Grid _reconstructed_mesh;
	Triangle _new_triangle;
};