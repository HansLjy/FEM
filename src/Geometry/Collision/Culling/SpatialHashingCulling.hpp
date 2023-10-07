#pragma once

#include "CCDCulling.hpp"

class SpatialHashingCulling : public CCDCulling {
public:
	void GetCCDSet(double d_hat, const std::vector<CollisionShapeInterface *> &objs, const std::vector<int> &offsets, const std::vector<int> &dofs, std::vector<PrimitivePair> &ccd_set) override;

protected:
	SpatialHashing<EdgePrimitiveInfo> _edge_hash_table;
	SpatialHashing<VertexPrimitiveInfo> _vertex_hash_table;

	unsigned int _time_stamp = 0;

};