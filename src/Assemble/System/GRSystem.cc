#include "GRSystem.hpp"
#include "FileIO.hpp"
#include <queue>

void CustomGrid::AddDanglingVertex(
	const Vector3d &dangling_vertex,
	int glue_id1, int glue_id2,
	double rest_length1, double rest_length2) {
	_has_dangling_vertex = true;
	_dangling_vertex = dangling_vertex;
	_glue_id[0] = glue_id1;
	_glue_id[1] = glue_id2;
	_rest_length[0] = rest_length1;
	_rest_length[1] = rest_length2;
}

void CustomGrid::MergeDanglingVertex(bool is_new_vertex) {
	assert(_has_dangling_vertex);
	if (is_new_vertex) {
		AddFace(_dangling_vertex);
	} else {
		AddFace();
	}
}

void CustomGrid::AddFace() {
	GridData::AddFace();
	_has_dangling_vertex = false;
}

void CustomGrid::AddFace(const Vector3d& position) {
	GridData::AddFace(position);
	_has_dangling_vertex = false;
}

Vector3d CustomGrid::GetVertex(int id) const {
	return CustomGrid::GetVertex(id, _x);
}

Vector3d CustomGrid::GetVertex(int id, const Ref<const VectorXd> &x) const {
	int grid_id = _vertices_grid_id(id);
	const Eigen::RowVector3d tri_coef = _trilinear_coef.row(id);
	const RowVector8i indices = _grid_topo.row(grid_id);
	
	Vector3d x_local = Vector3d::Zero();
	for (int id = 0; id < 8; id++) {	
		x_local += x.segment<3>(indices[id] * 3)
			* ((id & 1) ? tri_coef[0] : (1 - tri_coef[0]))
			* ((id & 2) ? tri_coef[1] : (1 - tri_coef[1]))
			* ((id & 4) ? tri_coef[2] : (1 - tri_coef[2]));
	}
	return x_local;
}

const BlockVector& CustomGrid::GetVertexDerivative(int id) const {
	return GetCollisionVertexDerivative(id);
}


GeometryReconstructSystem::GeometryReconstructSystem(const json& config)
	: _triangle_density(config["triangle-density"]),
	  _triangle_stiffness(config["triangle-stiffness"]),
	  _triangle_glue_stiffness(config["triangle-return-stiffness"]) {
	
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
	_prev_face.resize(_reconstructed_mesh->_face_topo.rows());
	GetNewTriangle();
	CalculateReturnPath(_father_face_id[_num_reconstructed_faces]);

	delete io;
};

GeometryReconstructSystem::~GeometryReconstructSystem() {
	delete _reconstructed_mesh;
	delete _new_triangle;
}

void GeometryReconstructSystem::AttachTriangle() {
	int free_vertex_cnt = 0;
	int free_vertex_id = 0;
	for (int i = 0; i < 3; i++) {
		if (_attach_vertex_id[i] == -1) {
			free_vertex_cnt++;
			free_vertex_id = i;
		}
	}
	assert(free_vertex_cnt < 2);
	switch (free_vertex_cnt) {
		case 0: {
			_reconstructed_mesh->AddDanglingVertex(
				_new_triangle->_x.segment<3>(6),
				_attach_vertex_id[0], _attach_vertex_id[1],
				_new_triangle->_rest_length[2], _new_triangle->_rest_length[1]
			);
			break;
		}
		case 1: {
			_reconstructed_mesh->AddDanglingVertex(
				_new_triangle->_x.segment<3>(free_vertex_id * 3),
				_attach_vertex_id[(free_vertex_id + 1) % 3],
				_attach_vertex_id[(free_vertex_id + 2) % 3],
				_new_triangle->_rest_length[free_vertex_id],
				_new_triangle->_rest_length[(free_vertex_id + 1) % 3]
			);
			break;
		}
	}
	_objs.pop_back();
	delete _new_triangle;
}

void GeometryReconstructSystem::MergeTriangle() {
	int free_vertex_cnt = 0;
	for (int i = 0; i < 3; i++) {
		free_vertex_cnt += (_attach_vertex_id[i] == -1);
	}
	_reconstructed_mesh->MergeDanglingVertex(free_vertex_cnt > 0);
}

void GeometryReconstructSystem::GetNewTriangle() {
	const RowVector3i indices = _reconstructed_mesh->_face_topo.row(_num_reconstructed_faces);
	Vector9d new_triangle_x_rest, new_triangle_x;
	for (int i = 0; i < 3; i++) {
		new_triangle_x_rest.segment<3>(i * 3) = _reconstructed_mesh->_x_rest.segment<3>(indices[i] * 3);
	}
	new_triangle_x = new_triangle_x_rest;
	new_triangle_x(2) += 2;
	new_triangle_x(5) += 2;
	new_triangle_x(8) += 2;

	_new_triangle = new CustomTriangle(new_triangle_x, new_triangle_x_rest, _triangle_density, _triangle_stiffness, _triangle_glue_stiffness);

	_objs.push_back(_new_triangle);
}

void GeometryReconstructSystem::UpdateLocalTarget(int face_id, int side) {
	_local_target_id = _prev_face[face_id];
	_local_target_orientation = _inverted_face[face_id] ? -side : side;
	const RowVector3i indicies = _reconstructed_mesh->_face_topo.row(_local_target_id);
	Vector3d x1 = _reconstructed_mesh->GetVertex(indicies[0]);
	Vector3d x2 = _reconstructed_mesh->GetVertex(indicies[1]);
	Vector3d x3 = _reconstructed_mesh->GetVertex(indicies[2]);
	_local_target_position = (x1 + x2 + x3) / 3 + _local_target_orientation * (x2 - x1).cross(x3 - x1).normalized() * _crawling_distance;
}

void GeometryReconstructSystem::Preprocessing(VectorXd &vertices, MatrixXi &topo) {
	const int num_points = vertices.size() / 3;
	const int num_faces = topo.rows();

	// Consider non-manifold edge
	std::vector<std::map<int, std::vector<int>>> edge2face(num_points);
	std::vector<std::vector<int>> tmp_neighbor_faces;
	std::vector<std::vector<bool>> tmp_is_neighbor_face_inverted;
	for (int i = 0; i < num_faces; i++) {
		const RowVector3i indices = topo.row(i);
		// check the neighbor of every edge
		for (int j = 0; j < 3; j++) {
			int id1 = indices[j];
			int id2 = indices[(j + 1) & 3];

			// Find neighbor face with the opposite direction
			auto itr = edge2face[id1].find(id2);
			if (itr != edge2face[id1].end()) {
				for (const auto& neighbor_id : itr->second) {
					tmp_neighbor_faces[i].push_back(neighbor_id);
					tmp_is_neighbor_face_inverted[i].push_back(true);
				}
				itr->second.push_back(i);
			} else {
				edge2face[id1].insert(std::make_pair(id2, std::vector<int>{i}));
			}
			
			// Find neighbor face with the same direction
			itr = edge2face[id2].find(id1);
			if (itr != edge2face[id2].end()) {
				for (const auto& neighbor_id : itr->second) {
					tmp_neighbor_faces[i].push_back(neighbor_id);
					tmp_is_neighbor_face_inverted[i].push_back(false);
				}
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
		_is_neighbor_face_inverted.push_back(tmp_is_neighbor_face_inverted[face_new_id2old_id[i]]);
	}

	vertex_visited = std::vector<bool>(num_points, false);
	_attach_vertex_ids.resize(num_faces, 3);
	for (int i = 0; i < 3; i++) {
		vertex_visited[topo(0, i)] = true;
	}
	for (int i = 1; i < num_faces; i++) {
		const RowVector3i vertex_ids = topo.row(i);
		for (int j = 0; j < 3; j++) {
			if (!vertex_visited[vertex_ids(j)]) {
				_attach_vertex_ids(i, j) = -1;
				vertex_visited[vertex_ids(j)] = true;
			} else {
				_attach_vertex_ids(i, j) = vertex_ids(j);
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
		int local_neighbor_id = 0;
		for (const auto neighbor_face_id : _neighbor_faces[cur_id]) {
			if (neighbor_face_id < _num_reconstructed_faces && !vertex_visited[neighbor_face_id]) {
				_prev_face[neighbor_face_id] = cur_id;
				_inverted_face[neighbor_face_id] = _is_neighbor_face_inverted[cur_id][local_neighbor_id];
				vertex_visited[neighbor_face_id] = true;
				face_ids.push(neighbor_face_id);
			}
			local_neighbor_id++;
		}
	}
}
