//
// Created by hansljy on 11/23/22.
//

#ifndef FEM_SPATIALHASHING_H
#define FEM_SPATIALHASHING_H

#include "EigenAll.h"
#include "JsonUtil.h"

template <typename NodeInfo>
class SpatialHashing {
public:
    SpatialHashing(double grid_length, unsigned int hash_table_size)
        : _grid_length(grid_length), _hash_table_size(hash_table_size), _hash_table(hash_table_size) {}

    void Insert(const Vector3d& position, const NodeInfo& info, int time_stamp) {
		const auto discrete_position = GetDiscretePosition(position);
        HashTableInsert(discrete_position, info, time_stamp);
    }

    void Insert(const Vector3d& bb_min, const Vector3d& bb_max, const NodeInfo& info, int time_stamp) {
        Vector<unsigned int, 3> bb_min_discrete = GetDiscretePosition(bb_min);
        Vector<unsigned int, 3> bb_max_discrete = GetDiscretePosition(bb_max) + Vector<unsigned int, 3>::Constant(1);

        for (unsigned int x = bb_min_discrete.x(); x != bb_max_discrete.x(); x++) {
            for (unsigned int y = bb_min_discrete.y(); y != bb_max_discrete.y(); y++) {
                for (unsigned int z = bb_min_discrete.z(); z != bb_max_discrete.z(); z++) {
					Vector<unsigned int, 3> discrete_position = (Vector<unsigned int, 3>() << x, y, z).finished();
                    HashTableInsert(discrete_position, info, time_stamp);
                }
            }
        }
    }

    std::vector<NodeInfo> Find(const Vector3d& bb_min, const Vector3d& bb_max, int time_stamp) {
        std::vector<NodeInfo> infos;

        Vector<unsigned int, 3> bb_min_discrete = GetDiscretePosition(bb_min);
        Vector<unsigned int, 3> bb_max_discrete = GetDiscretePosition(bb_max) + Vector<unsigned int, 3>::Constant(1);

        for (unsigned int x = bb_min_discrete.x(); x != bb_max_discrete.x(); x++) {
            for (unsigned int y = bb_min_discrete.y(); y != bb_max_discrete.y(); y++) {
                for (unsigned int z = bb_min_discrete.z(); z != bb_max_discrete.z(); z++) {
					// TODO: change it into sorted array merge
					Vector<unsigned int, 3> discrete_position = (Vector<unsigned int, 3>() << x, y, z).finished();
                    HashTableFind(discrete_position, time_stamp, infos);
                }
            }
        }

        return infos;
    }

protected:
	struct NodeInfoWithPos {
		Vector<unsigned int, 3> _discrete_position;
		NodeInfo _info;
	};

    struct HashTableEntry {
        int time_stamp;
        std::vector<NodeInfoWithPos> list;
    };

    Vector<unsigned int, 3> GetDiscretePosition(const Vector3d& position) {
        return (Vector<unsigned int, 3>()
                << floor(position.x() / _grid_length),
                   floor(position.y() / _grid_length),
                   floor(position.z() / _grid_length)
               ).finished();
    }

    unsigned int HashValue(const Vector<unsigned int, 3>& position) {
        unsigned int hash_value = ((position.x() * px) ^ (position.y() * py) ^ (position.z() * pz)) % _hash_table_size;
        return hash_value;
    }

    void HashTableInsert(const Vector<unsigned int, 3>& discrete_position, const NodeInfo& info, int time_stamp) {
		unsigned int hash_value = HashValue(discrete_position);
        auto& hash_entry = _hash_table[hash_value];
        if (!hash_entry.list.empty() && hash_entry.time_stamp != time_stamp) {
            hash_entry.list.clear();
        }
        hash_entry.time_stamp = time_stamp;
        hash_entry.list.push_back({discrete_position, info});
    }

    void HashTableFind(const Vector<unsigned int, 3>& discrete_position, int time_stamp, std::vector<NodeInfo>& infos) {
		unsigned int hash_value = HashValue(discrete_position);
        const auto& hash_entry = _hash_table[hash_value];
        if (hash_entry.time_stamp == time_stamp) {
			for (const auto& info_with_pos : hash_entry.list) {
				if (info_with_pos._discrete_position == discrete_position &&
					std::find(infos.begin(), infos.end(), info_with_pos._info) == infos.end()) {
					infos.push_back(info_with_pos._info);
				}
			}
        }
    }

    static const int px = 73856093;
    static const int py = 19349663;
    static const int pz = 83492791;

    double _grid_length;
    unsigned int _hash_table_size;
    std::vector<HashTableEntry> _hash_table;
};

#endif //FEM_SPATIALHASHING_H
