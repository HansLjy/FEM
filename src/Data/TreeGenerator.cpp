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
    Trunk(int segments, double length, double density, double youngs_module,
          double radius_max, double radius_min)
        : _segments(segments), _density(density), _youngs_module(youngs_module), _radius_max(radius_max), _radius_min(radius_min) {
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
        object["youngs-module"] = _youngs_module;
        object["radius-max"] = _radius_max;
        object["radius-min"] = _radius_min;
        object["root"] = {0, 0, -1};
        object["control-points"] = _control_points;
        object["segments"] = _segments;
        return object;
    }

    int _segments;
    double _density;
    double _youngs_module;
    double _radius_max, _radius_min;
    std::vector<double> _control_points;
};

struct Leaf {
    Leaf(double density, double thickness, double k_stretch, double k_shear, double k_bend_max, double k_bend_min,
         int u_segments, int v_segments, double long_axis, double short_axis)
         : _density(density), _thickness(thickness), _k_stretch(k_stretch), _k_shear(k_shear),
           _k_bend_max(k_bend_max), _k_bend_min(k_bend_min),
           _u_segments(u_segments), _v_segments(v_segments) {
        _control_points = {
            0,                  0,                 0,
            0, - short_axis * 0.8,     long_axis / 4,
            0,        -short_axis,     long_axis / 2,
            0,   short_axis * 0.8,     long_axis / 4,
            0,                  0,     long_axis / 2,
            0, - short_axis * 0.8, 3 * long_axis / 4,
            0,         short_axis,     long_axis / 2,
            0,   short_axis * 0.8, 3 * long_axis / 4,
            0,                  0,         long_axis
        };
    }

    operator json() {
        json leaf;
        leaf["type"] = "leaf";
        leaf["name"] = "leaf";
        leaf["density"] = _density;
        leaf["k-stretch"] = _k_stretch;
        leaf["k-shear"] = _k_shear;
        leaf["k-bend-max"] = _k_bend_max;
        leaf["k-bend-min"] = _k_bend_min;
        leaf["u-segments"] = _u_segments;
        leaf["v-segments"] = _v_segments;
        leaf["control-points"] = _control_points;
        leaf["thickness"] = _thickness;
        return leaf;
    }

    double _density, _thickness;
    double _k_stretch, _k_shear, _k_bend_max, _k_bend_min;
    int _u_segments, _v_segments;
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
double trunk_density;
double leaf_density;
double youngs_module;
double radius_root_max;
double radius_root_min;
double leaf_short_axis;
double leaf_long_axis;
double leaf_thickness;
int trunk_segments;
int leaf_segments;
double k_stretch, k_shear, k_bend_max, k_bend_min;
Vector3d g;

json DFS(int cur_level, double cur_length, double parent_radius_max, double parent_radius_min, double theta) {
    double distance_to_root = 0.6;
    double cur_radius_max = 0.5 * ((1 - distance_to_root) * parent_radius_max + distance_to_root * parent_radius_min);
    double cur_radius_min = cur_radius_max / parent_radius_max * parent_radius_min;

    double angle = 45;
//    Vector3d axis = (Vector3d() << Vector2d::Random().normalized(), 0).finished();
    Vector3d axis = (Vector3d() << cos(theta), sin(theta), 0).finished();

    json subdomain;
    subdomain["type"] = "tree-domain";
    subdomain["is-root"] = false;
    subdomain["position"]["distance-to-root"] = distance_to_root;
    subdomain["position"]["angle"] = angle;
    subdomain["position"]["axis"] = {axis(0), axis(1), axis(2)};

    if (cur_level == max_level) {
        subdomain["system"]["objects"] = {
                json(Leaf(leaf_density, leaf_thickness, k_stretch, k_shear, k_bend_max, k_bend_min, leaf_segments, leaf_segments, leaf_long_axis, leaf_short_axis))
        };
        subdomain["system"]["external-forces"] = {
                json(Gravity(g, "leaf"))
        };
    } else {
        subdomain["system"]["objects"] = {
                json(Trunk(trunk_segments, cur_length, trunk_density, youngs_module, cur_radius_max, cur_radius_min))
        };
        subdomain["system"]["external-forces"] = {
                json(Gravity(g, "trunk"))
        };
    }
    subdomain["system"]["constraints"] = json::array();

    subdomain["subdomains"] = json::array();
    if (cur_level < max_level) {
//        int num_children = rand() % max_fan_out;
        double current_angle = 0;
        double delta_angle = EIGEN_PI * 2 / max_fan_out;
        for (int i = 0; i < max_fan_out; i++, current_angle += delta_angle) {
            subdomain["subdomains"].push_back(
                DFS(cur_level + 1, cur_length * 0.5, cur_radius_max, cur_radius_min, current_angle)
            );
        }
    }
    return subdomain;
}

void GenerateTree(const std::string& config, const std::string& output_file) {
    std::ifstream config_file(config);
    json generator_config = json::parse(config_file);

    srand(generator_config["magic-number"]);

    max_level = generator_config["max-level"];
    root_length = generator_config["root-length"];
    max_fan_out = generator_config["max-fan-out"];
    trunk_density = generator_config["trunk-density"];
    leaf_density = generator_config["leaf-density"];
    youngs_module = generator_config["youngs-module"];
    radius_root_max = generator_config["radius-root-max"];
    radius_root_min = generator_config["radius-root-min"];
    trunk_segments = generator_config["trunk-segments"];
    g = Json2Vec(generator_config["g"]);

    k_stretch = generator_config["k-stretch"];
    k_shear = generator_config["k-shear"];
    k_bend_max = generator_config["k-bend-max"];
    k_bend_min = generator_config["k-bend-min"];
    leaf_segments = generator_config["leaf-segments"];

    leaf_short_axis = generator_config["leaf-short-axis"];
    leaf_long_axis = generator_config["leaf-long-axis"];
    leaf_thickness = generator_config["leaf-thickness"];

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
        json(Trunk(trunk_segments, root_length, trunk_density, youngs_module, radius_root_max, radius_root_min))
    };
    system["system"]["constraints"] = json::array();
    system["system"]["external-forces"] = {json(Gravity(g))};

    double current_angle = 0;
    double delta_angle = EIGEN_PI * 2 / max_fan_out;
    for (int i = 0; i < max_fan_out; i++, current_angle += delta_angle) {
        system["subdomains"].push_back(DFS(1, root_length * 0.5, radius_root_max, radius_root_min, current_angle));
    }

    std::ofstream output(output_file);
    output << system.dump();
}

int main() {
    GenerateTree(CONFIG_PATH "/generator.json", CONFIG_PATH "/model/tree-model.json");
}