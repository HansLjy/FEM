#include <fstream>
#include <string>
#include <spdlog/spdlog.h>
#include "ObjIO.hpp"
#include "Voxelizer/Voxelizer.hpp"
#include "gtest/gtest.h"
#include "JsonUtil.h"

TEST(VoxelizerTest, ContainmentTest) {
	std::fstream config_file(GEOMETRY_TEST_CONFIG_PATH "/voxelizer-test.json");
	json config;
	config_file >> config;

	const std::string obj_file_path = std::string("/") + std::string(config["obj-file"]);
	const double grid_size = config["grid-size"];

	ObjIO obj_io;
	VectorXd vertices;
	MatrixXi topo;
	obj_io.LoadFromFile(GEOMETRY_TEST_DATA_PATH + obj_file_path, vertices, topo);

	SimpleMeshVoxelizer voxlizer;
	VectorXd grid_vertices;
	MatrixXi grid_topo, edge_topo, face_topo;
	VectorXi vertices_grid_id;
	MatrixXd tri_coefs;
	voxlizer.Voxelize(
		vertices, topo, grid_size,
		grid_vertices, grid_topo, edge_topo, face_topo,
		vertices_grid_id, tri_coefs
	);
	spdlog::info("Voxelization finished, #grid = {}", grid_topo.rows());

	const int num_points = vertices.size() / 3;
	for (int i = 0; i < num_points; i++) {
		int grid_id = vertices_grid_id(i);
		Vector3d corner = grid_vertices.segment<3>(grid_topo(grid_id, 0) * 3);
		EXPECT_NEAR((corner + tri_coefs.row(i).transpose() * grid_size - vertices.segment<3>(i * 3)).norm(), 0, 1e-10);
	}

}