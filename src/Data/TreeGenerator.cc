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
double leaf_thickness;
double leaf_long_axis;
double leaf_short_axis;
int trunk_segments;
int leaf_segments;
double k_stretch, k_shear, k_bend_max, k_bend_min;
double position_ratio;
Vector3d g;

json GenerateTreeObject(int level, double angle, double cur_length, double current_radius_max, double current_radius_min, int fanout, int max_level) {
	json tree;
	
	if (level < max_level) {
		tree["type"] = "rigid-decomposed-treetrunk";
		tree["proxy"] = json(Trunk(trunk_segments, cur_length, trunk_density, youngs_module, current_radius_max, current_radius_min));
	} else {
		tree["type"] = "rigid-decomposed-leaf";
		tree["proxy"] = json(Leaf(leaf_density, leaf_thickness, k_stretch, k_shear, k_bend_max, k_bend_min, leaf_segments, leaf_segments, leaf_long_axis, leaf_short_axis));
	}

	if (level == 0) {
		tree["name"] = "treetrunk";
		tree["is-root"] = true;
		tree["x"] = {0, 0, 0};
		tree["rotation"] = {
			1, 0, 0,
			0, 1, 0,
			0, 0, 1
		};
	} else {
		tree["is-root"] = false;
		tree["position"]["angle"] = 45;
		tree["position"]["distance-to-root"] = position_ratio;

		const Vector3d vert = (Vector3d() << 0, 0, 1).finished();
		const Vector3d base_axis = (Vector3d() << 1, 0, 0).finished();
		Matrix3d rotation = Matrix3d(Eigen::AngleAxisd(angle, vert));
		Vector3d axis = rotation * base_axis;
		tree["position"]["axis"] = {axis(0), axis(1), axis(2)};
	}

	double cur_angle = 0;
	const double delta_angle = EIGEN_PI * 2 / fanout;
	const double next_radius_max = 0.5 * ((1 - position_ratio) * current_radius_max + position_ratio * current_radius_min);
	const double next_radius_min = next_radius_max / current_radius_max * current_radius_min;
	const double next_length = cur_length * 0.5;

	tree["children"] = json::array();
	if (level < max_level) {
		for (int i = 0; i < max_fan_out; i++, cur_angle += delta_angle) {
			tree["children"].push_back(GenerateTreeObject(level + 1, cur_angle, next_length, next_radius_max, next_radius_min, fanout, max_level));
		}
	}
	return tree;
}

void GenerateTree(const std::string& config) {
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

    leaf_thickness = generator_config["leaf-thickness"];
	leaf_long_axis = generator_config["leaf-long-axis"];
	leaf_short_axis = generator_config["leaf-short-axis"];
	position_ratio = generator_config["position-ratio"];

    json system;

	system["objects"] = json::array();
	system["objects"].push_back(GenerateTreeObject(0, 0, root_length, radius_root_max, radius_root_min, max_fan_out, max_level));

	system["external-forces"] = json::array();
	system["external-forces"].push_back(json(Gravity(g, "treetrunk")));

    std::ofstream output(CONFIG_PATH + std::string("/model/") + std::string(generator_config["filename"]));
    output << system.dump();
}

int main() {
    GenerateTree(CONFIG_PATH "/generator.json");
}