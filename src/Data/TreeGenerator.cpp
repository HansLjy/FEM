//
// Created by hansljy on 11/15/22.
//

#include "nlohmann/json.hpp"
#include "random"
#include "EigenAll.h"
#include "JsonUtil.h"
#include <fstream>
using nlohmann::json;

struct Trunk {
    Trunk(int segments, double length, double density, double alpha_max, double alpha_min,
          double radius_max, double radius_min, double k)
        : _segments(segments), _density(density), _alpha_max(alpha_max), _alpha_min(alpha_min), _radius_max(radius_max), _radius_min(radius_min), _k(k) {
        _control_points = {
                0, 0, 0,
                0, 0, length / 3,
                0, 0, 2 * length / 3,
                0, 0, length
        };
    }

    operator json() const {
        json object;
        object["type"] = "tree-trunk";
        object["name"] = "trunk";
        object["density"] = _density;
        object["alpha-max"] = _alpha_max;
        object["alpha-min"] = _alpha_min;
        object["radius-max"] = _radius_max;
        object["radius-min"] = _radius_min;
        object["k"] = _k;
        object["root"] = {0, 0, -1};
        object["control-points"] = _control_points;
        object["segments"] = _segments;
        return object;
    }

    int _segments;
    double _density;
    double _alpha_max, _alpha_min, _radius_max, _radius_min, _k;
    std::vector<double> _control_points;
};

struct Gravity {
    Gravity(const Vector3d& g, const std::string& name = "trunk") : _g(g), _name(name) {}

    operator json() const {
        json gravity;
        gravity["type"] = "sampled-object-gravity";
        gravity["object-name"] = _name;
        gravity["g"] = {_g(0), _g(1), _g(2)};
        return gravity;
    }

    Vector3d _g;
    std::string _name;
};

int max_level;
double root_length;
int max_fan_out;
double density;
double alpha_max;
double alpha_min;
double radius_root_max;
double radius_root_min;
double k;
int segments;
Vector3d g;

json DFS(int cur_level, double cur_length, double parent_radius_max, double parent_radius_min) {
    double distance_to_root = (rand() % 30 + 50) / 100.0;
    double cur_radius_max = (1 - distance_to_root) * parent_radius_max + distance_to_root * parent_radius_min;
    double cur_radius_min = cur_radius_max / parent_radius_max * parent_radius_min;

    double angle = rand() % 30 + 45;
    Vector3d axis = Vector3d::Random().normalized();

    json subdomain;
    subdomain["type"] = "tree-domain";
    subdomain["is-root"] = false;
    subdomain["position"]["distance-to-root"] = distance_to_root;
    subdomain["position"]["angle"] = angle;
    subdomain["position"]["axis"] = {axis(0), axis(1), axis(2)};

    subdomain["system"]["objects"] = {
        json(Trunk(segments, cur_length, density, alpha_max, alpha_min, cur_radius_max, cur_radius_min, k))
    };
    subdomain["system"]["constraints"] = json::array();
    subdomain["system"]["external-forces"] = {
        json(Gravity(g))
    };

    subdomain["subdomains"] = json::array();
    if (cur_level < max_level) {
        int num_children = rand() % max_fan_out;
        for (int i = 0; i < num_children; i++) {
            subdomain["subdomains"].push_back(
                DFS(cur_level + 1, cur_length * 0.5, cur_radius_max, cur_radius_min)
            );
        }
    }
    return subdomain;
}

void GenerateTree(const std::string& config, const std::string& output_file) {
    srand(time(NULL));

    std::ifstream config_file(config);
    json generator_config = json::parse(config_file);

    max_level = generator_config["max-level"];
    root_length = generator_config["root-length"];
    max_fan_out = generator_config["max-fan-out"];
    density = generator_config["density"];
    alpha_max = generator_config["alpha-max"];
    alpha_min = generator_config["alpha-min"];
    radius_root_max = generator_config["radius-root-max"];
    radius_root_min = generator_config["radius-root-min"];
    k = generator_config["k"];
    segments = generator_config["segments"];
    g = Json2Vec(generator_config["g"]);

    json system;
    system["type"] = "tree-domain";
    system["is-root"] = true;
    system["x"] = {0, 0, 0};
    system["rotation"] = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    };

    system["system"]["objects"] = {
        json(Trunk(segments, root_length, density, alpha_max, alpha_min, radius_root_max, radius_root_min, k))
    };
    system["system"]["constraints"] = json::array();
    system["system"]["external-forces"] = {json(Gravity(g))};

    for (int i = 0; i < max_fan_out; i++) {
        system["subdomains"].push_back(DFS(1, root_length * 0.5, radius_root_max, radius_root_min));
    }

    std::ofstream output(output_file);
    output << system.dump();
}

int main() {
    GenerateTree(CONFIG_PATH "/model/generator.json", CONFIG_PATH "/model/tree-model.json");
}