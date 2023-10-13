#include "InitUtils.hpp"
#include "unsupported/Eigen/KroneckerProduct"

VectorXd InitializationUtils::GenerateMass2D(const VectorXd& x, double density, const MatrixXi& topo) {
	int num_faces = topo.rows();
	VectorXd mass = VectorXd::Zero(x.size() / 3);
	for (int i = 0; i < num_faces; i++) {
		const auto& indices = topo.row(i);
		const int id1 = indices[0], id2 = indices[1], id3 = indices[2];

		const double mass_increment = density
			* (x.segment<3>(id2 * 3) - x.segment<3>(id1 * 3))
			  .cross(x.segment<3>(id3 * 3) - x.segment<3>(id1 * 3)).norm() / 2;
		const double single_mass_increment = mass_increment / 3;
		mass(id1) += single_mass_increment;
		mass(id2) += single_mass_increment;
		mass(id3) += single_mass_increment;
	}
	return mass;
}

VectorXd InitializationUtils::GenerateMass3D(const VectorXd& x, double density, const MatrixXi& topo) {
	int num_tets = topo.rows();
	VectorXd mass = VectorXd::Zero(x.size() / 3);
	for (int i = 0; i < num_tets; i++) {
		const auto& indices = topo.row(i);
		const int id1 = indices[0], id2 = indices[1], id3 = indices[2], id4 = indices[3];

		const double mass_increment = density
			* (x.segment<3>(id2 * 3) - x.segment<3>(id1 * 3))
			  .cross(x.segment<3>(id3 * 3) - x.segment<3>(id1 * 3))
			  .dot(x.segment<3>(id4 * 3) - x.segment<3>(id1 * 3));
		const double single_mass_increment = mass_increment / 4;
		mass(id1) += single_mass_increment;
		mass(id2) += single_mass_increment;
		mass(id3) += single_mass_increment;
		mass(id4) += single_mass_increment;
	}
	return mass;
}

VectorXd InitializationUtils::LinSpace(
	const Vector3d &start, const Vector3d &end, int num_segments
) {
    VectorXd x(3 * (num_segments + 1));
    Vector3d delta = (end - start) / num_segments;
    Vector3d current = start;
    for (int i = 0, j = 0; i <= num_segments; i++, j += 3, current += delta) {
        x.segment<3>(j) = current;
    }
    return x;
}

VectorXd InitializationUtils::LinSpace(
	double start, double end, int num_segments
) {
	VectorXd x(num_segments + 1);
	const double delta = (end - start) / num_segments;
	double cur = start;
	for (int i = 0; i <= num_segments; i++, cur += delta) {
		x(i) = cur;
	}
	return x;
}

VectorXd InitializationUtils::GenerateMass1D(const VectorXd &x, double density) {
    int num_points = x.size() / 3;
    VectorXd mass(num_points);
    mass.setZero();
    for (int i = 0, j = 0; i < num_points - 1; i++, j += 3) {
        Vector3d e = x.segment<3>(j + 3) - x.segment<3>(j);
        mass(i)     += e.norm() * density / 2;
        mass(i + 1) += e.norm() * density / 2;
    }
    return mass;
}

MatrixXi InitializationUtils::Curve::GenerateCurveEdgeTopo(int num_segments) {
	MatrixXi edge_topo(num_segments, 2);
	for (int i = 0; i < num_segments; i++) {
		edge_topo.row(i) << i, i + 1;
	}
	return edge_topo;
}

VectorXd InitializationUtils::BezierCurve::GenerateSamplePoints(const VectorXd &control_points, int num_segments) {
    VectorXd sample_points((num_segments + 1) * 3);
    double h = 1.0 / num_segments;
    for (int i = 0, j = 0; i <= num_segments; i++, j += 3) {
        double t = i * h;
        sample_points.segment<3>(j) =
            (1 - t) * (1 - t) * (1 - t) * control_points.segment<3>(0) +
            3 * (1 - t) * (1 - t) * t * control_points.segment<3>(3) +
            3 * (1 - t) * t * t * control_points.segment<3>(6) +
            t * t * t * control_points.segment<3>(9);
    }
    return sample_points;
}

SparseMatrixXd InitializationUtils::BezierCurve::GenerateBase(int num_segments) {
    SparseMatrixXd base((num_segments + 1) * 3, 12);
    COO coo;
    const double h = 1.0 / num_segments;
    for (int i = 0, ii = 0; i <= num_segments; i++, ii += 3) {
        double t = i * h;
        double B[4] = {
                (1 - t) * (1 - t) * (1 - t),
                3 * (1 - t) * (1 - t) * t,
                3 * (1 - t) * t * t,
                t * t * t
        };
        for (int j = 0, jj = 0; j < 4; j++, jj += 3) {
            for (int k = 0; k < 3; k++) {
                coo.push_back(Tripletd(ii + k, jj + k, B[j]));
            }
        }
    }
    base.setFromTriplets(coo.begin(), coo.end());
    return base;
}

VectorXd InitializationUtils::BezierCurve::GenerateShift(int num_segments) {
    return VectorXd::Zero(3 * (num_segments + 1));
}

VectorXd InitializationUtils::BezierSurface::GenerateSamplePoints(const VectorXd &control_points, int num_u_segments, int num_v_segments){
    SparseMatrixXd base = GenerateBase(num_u_segments, num_v_segments);
    return base * control_points;
}

MatrixXd InitializationUtils::BezierSurface::GenerateUVCoord(const VectorXd &control_points, int num_u_segments, int num_v_segments) {
    VectorXd x = GenerateSamplePoints(control_points, num_u_segments, num_v_segments);
    Vector3d lower_left = x.segment<3>(0);
    Vector3d u_dir = (x.segment<3>(3) - lower_left).normalized();
    Vector3d v_dir_unnormalized = x.segment<3>(3 * (num_u_segments + 1)) - lower_left;
    Vector3d v_dir = (v_dir_unnormalized - v_dir_unnormalized.dot(u_dir) * u_dir).normalized();

    const int num_sample_points = (num_u_segments + 1) * (num_v_segments + 1);
    VectorXd uv_coord(num_sample_points, 2);
    for (int i = 0, i3 = 0; i < num_sample_points; i++, i3 += 3) {
        Vector3d cur_x = x.segment<3>(i3) - lower_left;
        uv_coord.row(i) << cur_x.dot(u_dir), cur_x.dot(v_dir);
    }
    return uv_coord;
}

Vector2d InitializationUtils::BezierSurface::GenerateUVDir(const VectorXd &control_points, const Vector3d &dir) {
    Vector3d lower_left = control_points.segment<3>(0);
    Vector3d u_dir = (control_points.segment<3>(6) - lower_left).normalized();
    Vector3d v_dir = (control_points.segment<3>(18) - lower_left).normalized();
    return (Vector2d() << u_dir.dot(dir), v_dir.dot(dir)).finished();
}

SparseMatrixXd InitializationUtils::BezierSurface::GenerateBase(int num_u_segments, int num_v_segments) {
const int num_u_control_points = 3;
    const int num_v_control_points = 3;

    const int pascal_coef[] = {1, 2, 1};

    MatrixXd u_coeff(num_u_control_points, num_u_segments + 1);
    MatrixXd v_coeff(num_v_control_points, num_v_segments + 1);

    const double hu = 1.0 / num_u_segments;
    double ih = 0;
    for (int i = 0; i <= num_u_segments; i++, ih += hu) {
        for (int k = 0; k < 3; k++) {
            u_coeff(k, i) = pascal_coef[k] * pow(1 - ih, 2 - k) * pow(ih, k);
        }
    }
    const double hv = 1.0 / num_v_segments;
    double jh = 0;
    for (int i = 0; i <= num_v_segments; i++, jh += hv) {
        for (int k = 0; k < 3; k++) {
            v_coeff(k, i) = pascal_coef[k] * pow(1 - jh, 2 - k) * pow(jh, k);
        }
    }

    auto GetControlPointsIndex = [&, num_u_control_points] (int i, int j) {return num_u_control_points * j + i;};
    auto GetSamplePointsIndex = [&, num_u_segments] (int i, int j) {return (num_u_segments + 1) * j + i;};

    COO coo;
    for (int i = 0; i < num_u_control_points; i++) {
        for (int j = 0; j < num_v_control_points; j++) {
            for (int p = 0; p <= num_u_segments; p++) {
                for (int q = 0; q <= num_v_segments; q++) {
                    const double coef = u_coeff(i, p) * v_coeff(j, q);
                    const int control_point_offset = GetControlPointsIndex(i, j) * 3;
                    const int sample_point_offset = GetSamplePointsIndex(p, q) * 3;
                    for (int k = 0; k < 3; k++) {
                        coo.push_back(Tripletd(sample_point_offset + k, control_point_offset + k, coef));
                    }
                }
            }
        }
    }
    SparseMatrixXd hessian(3 * (num_u_segments + 1) * (num_v_segments + 1), 3 * num_u_control_points * num_v_control_points);
    hessian.setFromTriplets(coo.begin(), coo.end());

    return hessian;
}

VectorXd InitializationUtils::BezierSurface::GenerateShift(int num_u_segments, int num_v_segments) {
    return VectorXd::Zero(3 * (num_u_segments + 1) * (num_v_segments + 1));
}


VectorXd InitializationUtils::Surface::GenerateSquarePosition(
    const Vector3d &start,
    const Vector3d &u_end, const Vector3d &v_end,
    int num_u_segments, int num_v_segments
) {
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

MatrixXd InitializationUtils::Surface::GenerateSquareUVCoord(
    const Vector3d &start,
    const Vector3d &u_end, const Vector3d &v_end,
    int num_u_segments, int num_v_segments
) {
    Vector3d norm = (u_end - start).cross(v_end - start);
    Vector3d u_direction = (u_end - start).normalized();
    Vector3d v_direction = norm.cross(u_end - start).normalized();
    const Vector2d delta_u = (Vector2d() << (u_end - start).dot(u_direction), (u_end - start).dot(v_direction)).finished() / num_u_segments;
    const Vector2d delta_v = (Vector2d() << (v_end - start).dot(u_direction), (v_end - start).dot(v_direction)).finished() / num_v_segments;

    MatrixXd uv_coord((num_u_segments + 1) * (num_v_segments + 1), 2);
    for (int i = 0; i <= num_v_segments; i++) {
        for (int j = 0; j <= num_u_segments; j++) {
            uv_coord.row((i * (num_u_segments + 1) + j)) = i * delta_v + j * delta_u;
        }
    }
    return uv_coord;
}

MatrixXi InitializationUtils::Surface::GenerateSquareFaceTopo(
    int num_u_segments, int num_v_segments
) {
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

VectorXd InitializationUtils::Surface::GenerateSquareMass(
    double density, const VectorXd &x_rest, const MatrixXi &topo
) {
    int num_triangles = topo.rows();
    int num_points = x_rest.size() / 3;
    VectorXd mass(num_points);
    mass.setZero();
    for (int i = 0; i < num_triangles; i++) {
        RowVector3i indices = topo.row(i);
        const Vector3d e1 = x_rest.segment<3>(indices[1] * 3) - x_rest.segment<3>(indices[0] * 3);
        const Vector3d e2 = x_rest.segment<3>(indices[2] * 3) - x_rest.segment<3>(indices[0] * 3);
        const double area = e1.cross(e2).norm() / 2;
        for (int j = 0; j < 3; j++) {
            mass(indices[j]) += area * density / 3;
        }
    }
    return mass;
}

VectorXd InitializationUtils::Surface::CalculateArea(const VectorXd& x, const MatrixXi& face_topo) {
    const int num_faces = face_topo.rows();
    VectorXd area(num_faces);
    for (int i = 0; i < num_faces; i++) {
        RowVector3i index = face_topo.row(i);
        const Vector3d e1 = x.segment<3>(index[1] * 3) - x.segment<3>(index[0] * 3);
        const Vector3d e2 = x.segment<3>(index[2] * 3) - x.segment<3>(index[0] * 3);
        const double S = e1.cross(e2).norm() / 2;
        area(i) = S;
    }
    return area;
}

std::vector<Matrix2d> InitializationUtils::Surface::CalculateInv(
    const MatrixXd& uv_coord, const MatrixXi& face_topo
) {
    const int num_faces = face_topo.rows();
    std::vector<Matrix2d> inv;
    inv.reserve(num_faces);
    for (int i = 0; i < num_faces; i++) {
        RowVector3i index = face_topo.row(i);
        Matrix2d F;
        F.col(0) = (uv_coord.row(index(1)) - uv_coord.row(index(0))).transpose();
        F.col(1) = (uv_coord.row(index(2)) - uv_coord.row(index(0))).transpose();
        inv.emplace_back(F.inverse());
    }
    return inv;
}

std::vector<Matrix6d> InitializationUtils::Surface::CalculatePFPX(
    const MatrixXd& uv_coord, const MatrixXi& face_topo
) {
    const int num_faces = face_topo.rows();
    std::vector<Matrix6d> pFpx;
    pFpx.reserve(num_faces);
    for (int i = 0; i < num_faces; i++) {
        RowVector3i index = face_topo.row(i);
        Vector2d e1 = uv_coord.row(index(1)) - uv_coord.row(index(0));
        Vector2d e2 = uv_coord.row(index(2)) - uv_coord.row(index(0));

        Matrix2d F;
        F.col(0) = e1;
        F.col(1) = e2;

        pFpx.emplace_back(
            Eigen::KroneckerProduct<Matrix2d, Matrix3d>(
                F.inverse(), Matrix3d::Identity()
            )
        );
    }
    return pFpx;
}

