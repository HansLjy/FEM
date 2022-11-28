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
        auto hash_value = HashValue(position);
        const auto& hash_entry = _hash_table[hash_value];
        if (!hash_entry.empty() && hash_entry.time_stamp != time_stamp) {
            hash_entry.list.clear();
        }
        hash_entry.time_stamp = time_stamp;
        hash_entry.list.push_back(info);
    }
    const std::vector<NodeInfo>& Find(const Vector3d& position, int time_stamp) {
        auto hash_value = HashValue(position);
        const auto& hash_entry = _hash_table[hash_value];
        if (hash_entry.time_stamp != time_stamp) {
            return {};
        } else {
            return hash_entry.list;
        }
    }

protected:
    struct HashTableEntry {
        int time_stamp;
        std::vector<NodeInfo> list;
    };

    unsigned int HashValue(const Vector3d& position) {
        unsigned int x = floor(position.x() / _grid_length);
        unsigned int y = floor(position.y() / _grid_length);
        unsigned int z = floor(position.z() / _grid_length);
        unsigned int hash_value = ((x * px) ^ (y * py) ^ (z * pz)) % _hash_table_size;
        return hash_value;
    }

    static const int px = 73856093;
    static const int py = 19349663;
    static const int pz = 83492791;

    double _grid_length;
    unsigned int _hash_table_size;
    std::vector<HashTableEntry> _hash_table;
};

#endif //FEM_SPATIALHASHING_H
