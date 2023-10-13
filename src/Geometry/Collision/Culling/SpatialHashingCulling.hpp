#pragma once

#include "CCDCulling.hpp"
#include "SpatialQuery/SpatialHashing.hpp"

class SpatialHashingCulling : public CCDCulling {
public:
	void GetCCDSet(double d_hat, const std::vector<CollisionInterface> &objs, const std::vector<int> &offsets, std::vector<PrimitivePair> &ccd_set) override;

protected:
	SpatialHashing<EdgePrimitiveInfo> _edge_hash_table;
	SpatialHashing<VertexPrimitiveInfo> _vertex_hash_table;

	unsigned int _time_stamp = 0;

};