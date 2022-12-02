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
    explicit SpatialHashing(const json& config) : SpatialHashing(config["grid-length"], config["hash-table-size"]) {}

    SpatialHashing(double grid_length, unsigned int hash_table_size)
        : _grid_length(grid_length), _hash_table_size(hash_table_size), _hash_table(hash_table_size) {}

    void Insert(const Vector3d& position, const NodeInfo& info, int time_stamp) {
        auto hash_value = HashValue(GetDiscretePosition(position));
        HashTableInsert(hash_value, info, time_stamp);
    }

    void Insert(const Vector3d& bb_min, const Vector3d& bb_max, const NodeInfo& info, int time_stamp) {
        auto bb_min_discrete = GetDiscretePosition(bb_min);
        auto bb_max_discrete = GetDiscretePosition(bb_max);

        for (int x = bb_min_discrete.x(); x < bb_max_discrete.x(); x++) {
            for (int y = bb_min_discrete.y(); y < bb_max_discrete.y(); y++) {
                for (int z = bb_min_discrete.z(); z < bb_max_discrete.z(); z++) {
                    unsigned int hash_value = HashValue((Vector<unsigned int, 3>() << x, y, z).finished());
                    HashTableInsert(hash_value, info, time_stamp);
                }
            }
        }
    }

    std::vector<NodeInfo> Find(const Vector3d& bb_min, const Vector3d& bb_max, int time_stamp) {
        std::vector<NodeInfo> infos;

        auto bb_min_discrete = GetDiscretePosition(bb_min);
        auto bb_max_discrete = GetDiscretePosition(bb_max);

        for (int x = bb_min_discrete.x(); x < bb_max_discrete.x(); x++) {
            for (int y = bb_min_discrete.y(); y < bb_max_discrete.y(); y++) {
                for (int z = bb_min_discrete.z(); z < bb_max_discrete.z(); z++) {
                    unsigned int hash_value = HashValue((Vector<unsigned int, 3>() << x, y, z).finished());
                    HashTableFind(hash_value, time_stamp, infos);
                }
            }
        }

        return infos;
    }

protected:
    struct HashTableEntry {
        int time_stamp;
        std::vector<NodeInfo> list;
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

    void HashTableInsert(unsigned int hash_value, const NodeInfo& info, int time_stamp) {
        auto& hash_entry = _hash_table[hash_value];
        if (!hash_entry.list.empty() && hash_entry.time_stamp != time_stamp) {
            hash_entry.list.clear();
        }
        hash_entry.time_stamp = time_stamp;
        hash_entry.list.push_back(info);
    }

    void HashTableFind(unsigned int hash_value, int time_stamp, std::vector<NodeInfo>& info) {
        const auto& hash_entry = _hash_table[hash_value];
        if (hash_entry.time_stamp == time_stamp) {
            info.insert(info.end(), hash_entry.list.begin(), hash_entry.list.end());
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
