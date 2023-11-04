#include "CollisionInfo.hpp"
#include "CollisionInterface.hpp"
#include <iostream>

void DebugUtils::PrintPrimitivePair(const PrimitivePair& pair) {
	std::cerr << (pair._type == CollisionType::kEdgeEdge ? "Edge-Edge" : "Vertex-Face") << std::endl
			  << "Primitive 1: " << pair._obj_id1 << " " << pair._primitive_id1 << std::endl
			  << "Primitive 2: " << pair._obj_id2 << " " << pair._primitive_id2 << std::endl;	
}

void DebugUtils::PrintPrimitivePair(const PrimitivePair& pair, const std::vector<CollisionInterface>& objs) {
	PrintPrimitivePair(pair);
	if (pair._type == CollisionType::kEdgeEdge) {
		std::cerr << "vertices for edge 1: " << objs[pair._obj_id1].GetCollisionEdgeTopo().row(pair._primitive_id1) << std::endl;
		std::cerr << "vertices for edge 2: " << objs[pair._obj_id2].GetCollisionEdgeTopo().row(pair._primitive_id2) << std::endl;
	}
}
