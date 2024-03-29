#include <string>
#include <iostream>
#include "FileIO.hpp"
#include "Pattern.h"
#include "Voxelizer/Voxelizer.hpp"
#include "JsonUtil.h"
#include "GUI/GUI.hpp"

class VoxelizerTestGUI : public GUI {
public:
	explicit VoxelizerTestGUI(const json& config)
		: VoxelizerTestGUI(config["obj-file"], Json2Vec<3>(config["eye"]), config["grid-size"]) {}
	VoxelizerTestGUI(const std::string& obj_file, const Vector3d& eye, double grid_size)
		: GUI(true), _obj_file(obj_file), _grid_size(grid_size), _eye(eye) {}
	void InitializeScene(Scene &scene) override {
		SetCamera(
			glm::vec3(_eye(0), _eye(1), _eye(2)),
			glm::vec3(-1.0f, 0.0f, -1.0f),
			glm::vec3(0.0f, 0.0f, 1.0f)
		);
		SetLight(
			glm::vec3(0.0f, 10.0f, 0.0f),
			glm::vec3(0.2f, 0.2f, 0.2f),
			glm::vec3(0.5f, 0.5f, 0.5f),
			glm::vec3(1.0f, 1.0f, 1.0f),
			1.0f, 0.35f, 0.44f
		);

		FileIO* obj_io = Factory<FileIO>::GetInstance()->GetProduct("obj");
		VectorXd vertices;
		MatrixXi topo;
		obj_io->LoadFromFile(GEOMETRY_TEST_DATA_PATH + _obj_file, vertices, topo, false, Matrix3d::Identity(), Vector3d::Zero());
		vertices *= 10;	// stretch it

		SimpleMeshVoxelizer voxlizer;
		VectorXd grid_vertices;
		MatrixXi grid_topo, edge_topo, face_topo;
		VectorXi vertices_grid_id;
		MatrixXd tri_coefs;
		voxlizer.Voxelize(vertices, topo, _grid_size, grid_vertices, grid_topo, edge_topo, face_topo, vertices_grid_id, tri_coefs);

		scene.AddMesh();
		scene.SetTopo(topo);
		scene.SetMesh(StackVector<double, 3>(vertices), Matrix3d::Identity(), Vector3d::Zero());
		scene.SetBoundingBoxTopo(face_topo);
		scene.SetBoundingBoxMesh(StackVector<double, 3>(grid_vertices), Matrix3d::Identity(), Vector3d::Zero());
	}

	void Processing(Scene &scene) override {

	}

protected:
	std::string _obj_file;
	double _grid_size;
	Vector3d _eye;
};

int main() {
	std::fstream config_file(GEOMETRY_TEST_CONFIG_PATH "/voxelizer-test.json");
	json config;
	config_file >> config;
	VoxelizerTestGUI app(config);
	app.MainLoop();
}