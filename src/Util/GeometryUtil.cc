#include "GeometryUtil.h"

MatrixXi GetEdgeTopo(const MatrixXi& face_topo) {
	assert(face_topo.cols() == 3);
	const int num_faces = face_topo.rows();
	std::vector<std::pair<int, int>> edge_topo;
	for (int i = 0; i < num_faces; i++) {
		RowVector3i face = face_topo.row(i);
		for (int j = 0; j < 3; j++) {
			edge_topo.push_back(
				face(j) < face((j + 1) % 3)
				? std::make_pair(face(j), face((j + 1) % 3))
				: std::make_pair(face((j + 1) % 3), face(j))
			);
		}
	}

	std::sort(edge_topo.begin(), edge_topo.end());
	auto last = std::unique(edge_topo.begin(), edge_topo.end());
	edge_topo.erase(last, edge_topo.end());
	
	const int num_edges = edge_topo.size();
	MatrixXi edge_topo_mat(num_edges, 2);
	for (int i = 0; i < num_edges; i++) {
		edge_topo_mat.row(i) << edge_topo[i].first, edge_topo[i].second;
	}
	return edge_topo_mat;
}