#include "GRSystem.hpp"
#include "FileIO.hpp"
#include <queue>

GeometryReconstructSystem::GeometryReconstructSystem(const json& config)
	: _triangle_density(config["triangle-density"]),
	  _triangle_stiffness(config["triangle-stiffness"]),
	  _triangle_ret_stiffness(config["triangle-return-stiffness"]) {
	
	std::string filename = config["filename"];
	auto io = Factory<FileIO>::GetInstance()->GetProduct(GetSuffix(filename));
	VectorXd vertices;
	MatrixXi topo;
	io->LoadFromFile(filename, true, vertices, topo);
	Preprocessing(vertices, topo);

	_reconstructed_mesh = new CustomGrid(vertices, topo, 1, config["unit-length-stiffness"], config["unit-length-diag-stiffness"], config["unit-length-ret-stiffness"], config["grid-size"], config["grid-density"], config["render-bounding-box"]);
	_total_faces = _reconstructed_mesh->_num_faces;
	_num_reconstructed_faces = 1;
	_objs.reserve(2);
	_objs.push_back(_reconstructed_mesh);
	_prev_vertex.resize(_reconstructed_mesh->_face_topo.rows());
	GetNewTriangle();
	CalculateReturnPath(_father_face_id[_num_reconstructed_faces]);

	delete io;
};

GeometryReconstructSystem::~GeometryReconstructSystem() {
	delete _reconstructed_mesh;
	delete _new_triangle;
}

void GeometryReconstructSystem::GetNewTriangle() {
	delete _new_triangle;

	const RowVector3i indices = _reconstructed_mesh->_face_topo.row(_num_reconstructed_faces);
	Vector9d new_triangle_x_rest, new_triangle_x;
	for (int i = 0; i < 3; i++) {
		new_triangle_x_rest.segment<3>(i * 3) = _reconstructed_mesh->_x_rest.segment<3>(indices[i] * 3);
	}
	new_triangle_x = new_triangle_x_rest;
	new_triangle_x(2) += 2;
	new_triangle_x(5) += 2;
	new_triangle_x(8) += 2;

	_new_triangle = new CustomTriangle(new_triangle_x, new_triangle_x_rest, _triangle_density, _triangle_stiffness, _triangle_ret_stiffness);

	_objs[1] = _new_triangle;
}

void GeometryReconstructSystem::GlueTriangle() {
	int free_vertex_cnt = 0;
	int free_vertex_id = 0;
	for (int i = 0; i < 3; i++) {
		if (_glue_ids(_num_reconstructed_faces, i) == -1) {
			free_vertex_cnt++;
			free_vertex_id = i;
		}
	}
	assert(free_vertex_cnt < 2);
	switch (free_vertex_cnt) {
		case 0: {
			_reconstructed_mesh->AddFace();
			break;
		}
		case 1: {
			_reconstructed_mesh->AddFace(_new_triangle->_x.segment<3>(free_vertex_id * 3));
			break;
		}
	}
	_reconstructed_mesh->Update();
}

void GeometryReconstructSystem::Update() {
	GlueTriangle();
	GetNewTriangle();
	CalculateReturnPath(_father_face_id[_num_reconstructed_faces]);
	_num_reconstructed_faces++;
}

bool GeometryReconstructSystem::IsNear(double eps) const {
	const RowVector3i glue_ids = _glue_ids.row(_num_reconstructed_faces);
	for (int i = 0; i < 3; i++) {
		if (glue_ids[i] != -1) {
			const Vector3d x = _new_triangle->_x.segment<3>(i * 3);
			const Vector3d x_target = _reconstructed_mesh->_proxy->_x.segment<3>(glue_ids[i] * 3);
			if ((x - x_target).lpNorm<1>() > eps) {
				return false;
			}
		}
	}
	return true;
}

void GeometryReconstructSystem::Preprocessing(VectorXd &vertices, MatrixXi &topo) {
	const int num_points = vertices.size() / 3;
	const int num_faces = topo.rows();

	std::vector<std::map<int, int>> edge2face(num_points);
	std::vector<std::vector<int>> tmp_neighbor_faces;
	for (int i = 0; i < num_faces; i++) {
		const RowVector3i indices = topo.row(i);
		// check the neighbor of every edge
		for (int j = 0; j < 3; j++) {
			int id1 = std::min(indices[j], indices[(j + 1) & 3]);
			int id2 = std::max(indices[j], indices[(j + 1) & 3]);
			auto itr = edge2face[id1].find(id2);
			if (itr != edge2face[id1].end()) {
				tmp_neighbor_faces[i].push_back((*itr).second);
				edge2face[id1].erase(itr);
			}
		}
	}

	std::vector<int> father_face_id(num_faces);	// father in the BFS tree
	std::vector<bool> face_visited(num_faces, false);
	std::vector<int> face_new_id2old_id;
	face_new_id2old_id.reserve(num_faces);
	std::queue<int> face_ids;
	face_ids.push(0);
	face_visited[0] = true;
	father_face_id[0] = 0;
	while (!face_ids.empty()) {
		int cur_face_id = face_ids.front();
		face_new_id2old_id.push_back(cur_face_id);
		face_ids.pop();
		for (const auto& neighbor_id : tmp_neighbor_faces[cur_face_id]) {
			if (!face_visited[neighbor_id]) {
				face_visited[neighbor_id] = true;
				face_ids.push(neighbor_id);
				father_face_id[neighbor_id] = cur_face_id;
			}
		}
	}

	std::vector<bool> vertex_visited(num_points, false);
	std::vector<int> vertex_new_id2old_id;
	vertex_new_id2old_id.reserve(num_points);
	for (const int face_id : face_new_id2old_id) {
		const RowVector3i vertex_indices = topo.row(face_id);
		for (int i = 0; i < 3; i++) {
			if (!vertex_visited[vertex_indices[i]]) {
				vertex_new_id2old_id.push_back(vertex_indices[i]);
				vertex_visited[vertex_indices[i]] = true;
			}
		}
	}

	std::vector<int> vertex_old_id2new_id(num_points);
	for (int i = 0; i < num_points; i++) {
		vertex_old_id2new_id[vertex_new_id2old_id[i]] = i;
	}

	std::vector<int> face_old_id2new_id(num_faces);
	for (int i = 0; i < num_faces; i++) {
		face_old_id2new_id[face_new_id2old_id[i]] = i;
	}

	MatrixXi corrected_topo;
	corrected_topo.resizeLike(topo);
	for (int i = 0; i < num_faces; i++) {
		const RowVector3i vertex_ids = topo.row(face_new_id2old_id[i]);
		corrected_topo.row(i) << vertex_old_id2new_id[vertex_ids(0)], vertex_old_id2new_id[vertex_ids(1)], vertex_old_id2new_id[vertex_ids(2)];
	}
	topo = corrected_topo;

	VectorXd corrected_vertices;
	for (int i = 0; i < num_points; i++) {
		corrected_vertices.segment<3>(i * 3) = corrected_vertices.segment<3>(vertex_new_id2old_id[i] * 3);
	}
	vertices = corrected_vertices;

	for (int i = 0; i < num_faces; i++) {
		const auto& neighbor_face_ids = tmp_neighbor_faces[face_new_id2old_id[i]];
		std::vector<int> corrected_neighbor_face_ids;
		for (const auto& neighbor_face_id : neighbor_face_ids) {
			corrected_neighbor_face_ids.push_back(face_old_id2new_id[neighbor_face_id]);
		}
		_neighbor_faces.push_back(corrected_neighbor_face_ids);
	}

	vertex_visited = std::vector<bool>(num_points, false);
	_glue_ids.resize(num_faces, 3);
	for (int i = 0; i < 3; i++) {
		vertex_visited[topo(0, i)] = true;
	}
	for (int i = 1; i < num_faces; i++) {
		const RowVector3i vertex_ids = topo.row(i);
		for (int j = 0; j < 3; j++) {
			if (!vertex_visited[vertex_ids(j)]) {
				_glue_ids(i, j) = -1;
				vertex_visited[vertex_ids(j)] = true;
			} else {
				_glue_ids(i, j) = vertex_ids(j);
			}
		}
	}

	_father_face_id.resize(num_faces);
	for (int i = 0; i < num_faces; i++) {
		_father_face_id[face_old_id2new_id[i]] = face_new_id2old_id[father_face_id[i]];
	}
}

void GeometryReconstructSystem::CalculateReturnPath(int source_face_id) {
	std::queue<int> face_ids;
	std::vector<bool> vertex_visited(_num_reconstructed_faces, false);
	face_ids.push(source_face_id);
	while(!face_ids.empty()) {
		int cur_id = face_ids.front();
		face_ids.pop();
		for (const auto neighbor_face_id : _neighbor_faces[cur_id]) {
			if (neighbor_face_id < _num_reconstructed_faces && !vertex_visited[neighbor_face_id]) {
				_prev_vertex[neighbor_face_id] = cur_id;
				vertex_visited[neighbor_face_id] = true;
				face_ids.push(neighbor_face_id);
			}
		}		
	}
}
