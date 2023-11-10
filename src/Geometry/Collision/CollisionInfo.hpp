#pragma once

#include "EigenAll.h"

enum class CollisionType {
	kVertexFace,
	kEdgeEdge
};

struct PrimitivePair {
	CollisionType _type;
	int _obj_id1;
	int _obj_id2;
	int _primitive_id1;
	int _primitive_id2;

	bool operator<(const PrimitivePair& rhs) const {
		if (_type != rhs._type) {
			return _type < rhs._type;
		}
		if (_obj_id1 != rhs._obj_id1) {
			return _obj_id1 < rhs._obj_id1;
		}
		if (_obj_id2 != rhs._obj_id2) {
			return _obj_id2 < rhs._obj_id2;
		}
		if (_primitive_id1 != rhs._primitive_id1) {
			return _primitive_id1 < rhs._primitive_id1;
		}
		return _primitive_id2 < rhs._primitive_id2;
	}
};

#include <iostream>

class CollisionInterface;
namespace DebugUtils {
	void PrintPrimitivePair(const PrimitivePair& pair);

	template<class CollisionInterface>
	void PrintPrimitivePair(const PrimitivePair& pair, const std::vector<CollisionInterface>& objs) {
		PrintPrimitivePair(pair);
		if (pair._type == CollisionType::kEdgeEdge) {
			std::cerr << "vertices for edge 1: " << objs[pair._obj_id1].GetCollisionEdgeTopo().row(pair._primitive_id1) << std::endl;
			std::cerr << "vertices for edge 2: " << objs[pair._obj_id2].GetCollisionEdgeTopo().row(pair._primitive_id2) << std::endl;
		} else {
			std::cerr << "vertices for edge 1: " << pair._primitive_id1 << std::endl;
			std::cerr << "vertices for edge 2: " << objs[pair._obj_id2].GetCollisionFaceTopo().row(pair._primitive_id2) << std::endl;
		}
	}

	inline bool PrimitivePairEqual(const PrimitivePair& primitive_pair, int obj_id1, int primitive_id1, int obj_id2, int primitive_id2) {
		return primitive_pair._obj_id1 == obj_id1
			&& primitive_pair._obj_id2 == obj_id2
			&& primitive_pair._primitive_id1 == primitive_id1
			&& primitive_pair._primitive_id2 == primitive_id2;
	}
}

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