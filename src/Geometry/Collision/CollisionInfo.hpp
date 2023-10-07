#pragma once

#include "EigenAll.h"

enum class CollisionType {
	kVertexFace,
	kEdgeEdge
};

struct CollisionInfo {
	CollisionType _type;
	double _distance;
	int _obj_id1;
	int _obj_id2;
	int _primitive_id1;
	int _primitive_id2;
};

struct PardonPairInfo {
	int _obj_id1;
	int _obj_id2;
	int _primitive_id1;
	int _primitive_id2;
};

struct EdgePrimitiveInfo {
    int _obj_id;
    int _primitive_id;
	Vector3d _vertex1;
	Vector3d _vertex2;

	bool operator<(const EdgePrimitiveInfo& rhs) const {
		return _obj_id < rhs._obj_id || (_obj_id == rhs._obj_id && _primitive_id < rhs._primitive_id);
	}

	bool operator==(const EdgePrimitiveInfo& rhs) const {
		return _obj_id == rhs._obj_id && _primitive_id == rhs._primitive_id;
	}
};

struct VertexPrimitiveInfo {
    int _obj_id;
    int _primitive_id;
	Vector3d _vertex;

	bool operator<(const VertexPrimitiveInfo& rhs) const {
		return _obj_id < rhs._obj_id || (_obj_id == rhs._obj_id && _primitive_id < rhs._primitive_id);
	}

	bool operator==(const VertexPrimitiveInfo& rhs) const {
		return _obj_id == rhs._obj_id && _primitive_id == rhs._primitive_id;
	}
};