//
// Created by hansljy on 11/23/22.
//

#include "SimpleCCD.h"
#include "CubicSolver/CubicSolver.h"

SimpleCCD* SimpleCCD::CreateFromConfig(const json &config) {
	return new SimpleCCD(config["epsilon"], CubicSolver::GetProductFromConfig(config["cubic-solver"]));
}

double SimpleCCD::EdgeEdgeCollision(const Eigen::Vector3d &x11, const Eigen::Vector3d &x12,
                                    const Eigen::Vector3d &x21, const Eigen::Vector3d &x22,
                                    const Eigen::Vector3d &v11, const Eigen::Vector3d &v12,
                                    const Eigen::Vector3d &v21, const Eigen::Vector3d &v22) {
    Vector3d dx1 = x12 - x11, dx2 = x21 - x11, dx3 = x22 - x11;
    Vector3d dv1 = v12 - v11, dv2 = v21 - v11, dv3 = v22 - v11;

    double A = dv1.cross(dv2).dot(dv3);
    double B = dx1.cross(dv2).dot(dv3) + dv1.cross(dx2).dot(dv3) + dv1.cross(dv2).dot(dx3);
    double C = dx1.cross(dx2).dot(dv3) + dx1.cross(dv2).dot(dx3) + dv1.cross(dx2).dot(dx3);
    double D = dx1.cross(dx2).dot(dx3);

    double t = _cubic_solver->Solve(A, B, C, D, 0, 1);

	if (t >= 0 && CheckEdgeIntersection(x11 + t * v11, x12 + t * v12, x21 + t * v21, x22 + t * v22)) {
		return t;
	} else {
		return 2;
	}
}

double SimpleCCD::VertexFaceCollision(const Eigen::Vector3d &x,
                                      const Eigen::Vector3d &x1, const Eigen::Vector3d &x2, const Eigen::Vector3d &x3,
                                      const Eigen::Vector3d &v,
                                      const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, const Eigen::Vector3d &v3) {
    Vector3d dx1 = x - x1, dx2 = x - x2, dx3 = x - x3;
    Vector3d dv1 = v - v1, dv2 = v - v2, dv3 = v - v3;

    double A = dv1.cross(dv2).dot(dv3);
    double B = dx1.cross(dv2).dot(dv3) + dv1.cross(dx2).dot(dv3) + dv1.cross(dv2).dot(dx3);
    double C = dx1.cross(dx2).dot(dv3) + dx1.cross(dv2).dot(dx3) + dv1.cross(dx2).dot(dx3);
    double D = dx1.cross(dx2).dot(dx3);

    double t = _cubic_solver->Solve(A, B, C, D, 0, 1);
    if (t >= 0 && CheckVertexInFace(x + t * v, x1 + t * v1, x2 + t * v2, x3 + t * v3)) {
        return t;
    } else {
        return 2;
    }
}

bool
SimpleCCD::CheckVertexInFace(const Eigen::Vector3d &vertex, const Eigen::Vector3d &face1, const Eigen::Vector3d &face2, const Eigen::Vector3d &face3) {
    Matrix<double, 3, 2> A;
	A.col(0) = face2 - face1;
	A.col(1) = face3 - face1;
    Eigen::FullPivLU<Matrix<double, 3, 2>> lu_decomp(A);
	// we don't need to check the rank, unless the triangle itself degenerates
	Vector2d sol = lu_decomp.solve(vertex - face1);
	return sol(0) >= 0 && sol(1) >= 0 && sol(0) + sol(1) <= 1;
}

//<- https://math.stackexchange.com/questions/3114665/how-to-find-if-two-line-segments-intersect-in-3d
bool SimpleCCD::CheckEdgeIntersection(const Eigen::Vector3d &edge11, const Eigen::Vector3d &edge12, const Eigen::Vector3d &edge21, const Eigen::Vector3d &edge22) {
	Vector3d e1 = edge11 - edge12;
    Vector3d e2 = - edge21 + edge22;

	if (e1.normalized().cross(e2.normalized()).norm() < 1e-10) {
		return false;
	} else {
    	Matrix<double, 3, 2> A;
		A.col(0) = e1;
		A.col(1) = e2;
    	Eigen::FullPivLU<Matrix<double, 3, 2>> lu_decomp(A);
		Vector2d sol = lu_decomp.solve(edge22 - edge12);
		return sol(0) >= 0 && sol(0) <= 1 && sol(1) >= 0 && sol(1) <= 1;
	}
}

SimpleCCD::~SimpleCCD() {
    delete _cubic_solver;
}
