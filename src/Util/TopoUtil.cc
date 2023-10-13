#include "TopoUtil.hpp"

MatrixXi TopoUtil::GetFaceTopo(const MatrixXi& tet_topo) {
	// TODO:
}

MatrixXi TopoUtil::GetEdgeTopo(const MatrixXi& face_topo) {
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

void TopoUtil::GenerateSurfaceTopo3D(const MatrixXi& tet_topo, MatrixXi& face_topo, MatrixXi& edge_topo) {
	face_topo = GetFaceTopo(tet_topo);
	edge_topo = GetEdgeTopo(face_topo);
}

void TopoUtil::GenerateSurfaceTopo2D(const MatrixXi& face_topo, MatrixXi& edge_topo) {
	edge_topo = GetEdgeTopo(face_topo);
}

MatrixXi TopoUtil::GetInternalEdge(const MatrixXi &face_topo) {
	const int num_faces = face_topo.rows();
	std::vector<std::tuple<int, int, int>> edges;

    for (int i = 0; i < num_faces; i++) {
        RowVector3i indices = face_topo.row(i);

        for (int j = 0; j < 3; j++) {
            edges.push_back(
                std::make_tuple(
                    std::min(indices(j), indices((j + 1) % 3)),
                    std::max(indices(j), indices((j + 1) % 3)),
                    indices((j + 2) % 3)
                )
            );
        }
    }

    std::sort(edges.begin(), edges.end());
    const int num_edges = edges.size();
    int num_internal_edges = 0;
    for (int i = 0; i < num_edges - 1; i++) {
        if (std::get<0>(edges[i]) == std::get<0>(edges[i + 1])
            && std::get<1>(edges[i]) == std::get<1>(edges[i + 1])) {
            // internal edge
            num_internal_edges++;
        }
    }
	
    MatrixXi internal_edge(num_internal_edges, 4);

    num_internal_edges = 0;
    for (int i = 0; i < num_edges - 1; i++) {
        if (std::get<0>(edges[i]) == std::get<0>(edges[i + 1])
            && std::get<1>(edges[i]) == std::get<1>(edges[i + 1])) {
            internal_edge.row(num_internal_edges)
                << std::get<0>(edges[i]), std::get<1>(edges[i]),
                   std::get<2>(edges[i]), std::get<2>(edges[i + 1]);
            num_internal_edges++;
        }
    }

	return internal_edge;
}
