#include "ClothData.hpp"
#include "ReducedDataUtils.hpp"
#include "unsupported/Eigen/KroneckerProduct"

VectorXd ClothData::GeneratePosition(const Eigen::Vector3d &start, const Eigen::Vector3d &u_end, const Eigen::Vector3d &v_end, int num_u_segments, int num_v_segments) {
    Vector3d delta_u = (u_end - start) / num_u_segments;
    Vector3d delta_v = (v_end - start) / num_v_segments;
    VectorXd position(3 * (num_u_segments + 1) * (num_v_segments + 1));

    /**
     *  v-end
     *     |
     *    U+1
     *     |
     *     0 -- 1 -- 2 -- ...
     *  start                u-end
     */
    for (int i = 0; i <= num_v_segments; i++) {
        for (int j = 0; j <= num_u_segments; j++) {
            position.segment<3>(3 * (i * (num_u_segments + 1) + j)) = start + j * delta_u + i * delta_v;
        }
    }
    return position;
}

VectorXd ClothData::GenerateUVCoord(const Eigen::Vector3d &start, const Eigen::Vector3d &u_end, const Eigen::Vector3d &v_end, int num_u_segments, int num_v_segments) {
    Vector3d norm = (u_end - start).cross(v_end - start);
    Vector3d u_direction = (u_end - start).normalized();
    Vector3d v_direction = norm.cross(u_end - start).normalized();
    const Vector2d delta_u = (Vector2d() << (u_end - start).dot(u_direction), (u_end - start).dot(v_direction)).finished() / num_u_segments;
    const Vector2d delta_v = (Vector2d() << (v_end - start).dot(u_direction), (v_end - start).dot(v_direction)).finished() / num_v_segments;

    VectorXd uv_coord(2 * (num_u_segments + 1) * (num_v_segments + 1));
    for (int i = 0; i <= num_v_segments; i++) {
        for (int j = 0; j <= num_u_segments; j++) {
            uv_coord.segment<2>(2 * (i * (num_u_segments + 1) + j)) = i * delta_v + j * delta_u;
        }
    }
    return uv_coord;
}

MatrixXi ClothData::GenerateTopo(int num_u_segments, int num_v_segments) {
    MatrixXi topo(num_u_segments * num_v_segments * 2, 3);
    for (int i = 0; i < num_v_segments; i++) {
        for (int j = 0; j < num_u_segments; j++) {
            // row i, column j
            const int lower_left_id = i * (num_u_segments + 1) + j;
            topo.row(2 * (i * num_u_segments + j))
                << lower_left_id, lower_left_id + 1, lower_left_id + num_u_segments + 1;
            topo.row(2 * (i * num_u_segments + j) + 1)
                << lower_left_id + 1, lower_left_id + num_u_segments + 1, lower_left_id + num_u_segments + 2;
        }
    }
    return topo;
}

VectorXd ClothData::GenerateMass(double rho, double thickness, const VectorXd &uv_coord, const MatrixXi &topo) {
    int num_triangles = topo.rows();
    int num_points = uv_coord.size() / 2;
    VectorXd mass(num_points);
    mass.setZero();
    for (int i = 0; i < num_triangles; i++) {
        RowVector3i index = topo.row(i);
        Vector2d e1 = uv_coord.segment<2>(index(1) * 2) - uv_coord.segment<2>(index(0) * 2);
        Vector2d e2 = uv_coord.segment<2>(index(2) * 2) - uv_coord.segment<2>(index(0) * 2);
        const double S = abs(e1(0) * e2(1) - e1(1) * e2(0)) / 2;
        for (int j = 0; j < 3; j++) {
            mass(index(j)) += S * rho * thickness / 3;
        }
    }
    return mass;
}


ClothData::ClothData(const json& config)
    : ClothData(
		config["density"], config["thickness"],
		config["k-stretch"], config["k-shear"], config["k-bend-max"], config["k-bend-min"],
		Json2Vec<2>(config["max-direction"]), Json2Vec(config["start"]),
		Json2Vec(config["u-end"]), Json2Vec(config["v-end"]), config["u-segments"], config["v-segments"],
		config["stretch-u"], config["stretch-v"]) {}


ClothData::ClothData(
	double rho, double thickness,
	double k_stretch, double k_shear, double k_bend_max, double k_bend_min,
	const Eigen::Vector2d &max_bend_dir,
	const Eigen::Vector3d &start, const Eigen::Vector3d &u_end, const Eigen::Vector3d &v_end,
	int num_u_segments, int num_v_segments, double stretch_u, double stretch_v)
    : ClothData(rho, thickness, k_stretch, k_shear, k_bend_max, k_bend_min, max_bend_dir,
                     GeneratePosition(start, u_end, v_end, num_u_segments, num_v_segments),
                     GenerateUVCoord(start, u_end, v_end, num_u_segments, num_v_segments),
                     GenerateTopo(num_u_segments, num_v_segments),
                     stretch_u, stretch_v) {}

ClothData::ClothData(
	double rho, double thickness,
	double k_stretch, double k_shear, double k_bend_max, double k_bend_min,
	const Eigen::Vector2d &max_bend_dir,
	const Eigen::VectorXd &x, const Eigen::VectorXd &uv_corrd,
	const Eigen::MatrixXi &topo,
	double stretch_u, double stretch_v)
	: SampledObjectData(x, GenerateMass(rho, thickness, uv_corrd, topo), 2, topo),
               _curve_num_points(x.size() / 3),
               _num_triangles(topo.rows()),
               _k_stretch(k_stretch), _k_shear(k_shear),
               _stretch_u(stretch_u), _stretch_v(stretch_v),
               _uv_coord(uv_corrd)
               {
    _area.resize(_num_triangles);
    for (int i = 0; i < _num_triangles; i++) {
        RowVector3i index = _face_topo.row(i);
        Vector2d e1 = uv_corrd.segment<2>(index(1) * 2) - uv_corrd.segment<2>(index(0) * 2);
        Vector2d e2 = uv_corrd.segment<2>(index(2) * 2) - uv_corrd.segment<2>(index(0) * 2);
        const double S = (e1(0) * e2(1) - e1(1) * e2(0)) / 2;
        if (S > 0) {
            _area(i) = S;
        } else {
            _area(i) = -S;
            std::swap(_face_topo(i, 1), _face_topo(i, 2));
        }
    }

    std::vector<std::tuple<int, int, int>> edges;

    _inv.resize(_num_triangles);
    _pFpx.resize(_num_triangles);
    for (int i = 0; i < _num_triangles; i++) {
        RowVector3i index = _face_topo.row(i);
        Vector2d e1 = uv_corrd.segment<2>(index(1) * 2) - uv_corrd.segment<2>(index(0) * 2);
        Vector2d e2 = uv_corrd.segment<2>(index(2) * 2) - uv_corrd.segment<2>(index(0) * 2);

        Matrix2d F;
        F.col(0) = e1;
        F.col(1) = e2;

        _inv(i) = F.inverse();
        _pFpx(i) = Eigen::KroneckerProduct<Matrix2d, Matrix3d>(_inv(i), Matrix3d::Identity());

        for (int j = 0; j < 3; j++) {
            edges.push_back(
                std::make_tuple(
                    std::min(index(j), index((j + 1) % 3)),
                    std::max(index(j), index((j + 1) % 3)),
                    index((j + 2) % 3)
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
    _internal_edge.resize(num_internal_edges, 4);
    _internal_edge_length.resize(num_internal_edges);
    _internal_edge_k_bend.resize(num_internal_edges);
    _num_internal_edges = 0;
    for (int i = 0; i < num_edges - 1; i++) {
        if (std::get<0>(edges[i]) == std::get<0>(edges[i + 1])
            && std::get<1>(edges[i]) == std::get<1>(edges[i + 1])) {
            _internal_edge.row(_num_internal_edges)
                << std::get<0>(edges[i]), std::get<1>(edges[i]),
                   std::get<2>(edges[i]), std::get<2>(edges[i + 1]);
            _internal_edge_length(_num_internal_edges) = (
                    _x.segment<3>(3 * std::get<1>(edges[i])) -
                    _x.segment<3>(3 * std::get<0>(edges[i]))
            ).norm();
            Vector2d uv_direction = _uv_coord.segment<2>(2 * std::get<1>(edges[i])) - _uv_coord.segment<2>(2 * std::get<0>(edges[i]));
            const double delta_u = uv_direction.dot(max_bend_dir);
            const double delta_v = - uv_direction(0) * max_bend_dir(1) + uv_direction(1) * max_bend_dir(0);
            _internal_edge_k_bend(_num_internal_edges) =
                    (k_bend_max * delta_u * delta_u + k_bend_min * delta_v * delta_v) /
                    (delta_u * delta_u + delta_v * delta_v);

            _num_internal_edges++;
        }
    }
}

BezierClothData::BezierClothData(const json &config)
    : BezierClothData (
        Json2VecX(config["control-points"]),
        config["density"], config["thickness"], config["k-stretch"], config["k-shear"], config["k-bend-max"], config["k-bend-min"], Json2Vec(config["max-dir"]),
        config["u-segments"], config["v-segments"], config["stretch-u"], config["stretch-v"]
      ) {}

BezierClothData::BezierClothData(
	const VectorXd &control_points,
	double rho, double thickness,
	double k_stretch, double k_shear, double k_bend_max, double k_bend_min,
	const Vector3d& max_dir,
	int num_u_segments, int num_v_segments,
	double stretch_u, double stretch_v)
	: ReducedObjectData<ClothData>(
		control_points,
		new ClothData(rho, thickness, k_stretch, k_shear, k_bend_max, k_bend_min,
                  ReducedDataUtils::BezierSurface::GenerateUVDir(control_points, max_dir),
                  ReducedDataUtils::BezierSurface::GenerateSamplePoints(control_points, num_u_segments, num_v_segments),
                  ReducedDataUtils::BezierSurface::GenerateUVCoord(control_points, num_u_segments, num_v_segments),
				  ClothData::GenerateTopo(num_u_segments, num_v_segments), stretch_u, stretch_v),
		ReducedDataUtils::BezierSurface::GenerateBase(num_u_segments, num_v_segments),
		ReducedDataUtils::BezierSurface::GenerateShift(num_u_segments, num_v_segments)
		) {}