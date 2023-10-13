#pragma once

#include "EigenAll.h"

namespace InitializationUtils {
    VectorXd GenerateMass2D(const VectorXd& x, double density, const MatrixXi& topo);
    VectorXd GenerateMass3D(const VectorXd& x, double density, const MatrixXi& topo);
    VectorXd GenerateMass1D(const VectorXd& x, double density);
    VectorXd LinSpace(const Vector3d &start, const Vector3d &end, int num_segments);
    VectorXd LinSpace(double start, double end, int num_segments);

    namespace Curve {
        MatrixXi GenerateCurveEdgeTopo(int num_segments);
    }

    namespace Surface {
        VectorXd GenerateSquarePosition(const Vector3d &start, const Vector3d &u_end, const Vector3d &v_end, int num_u_segments, int num_v_segments);
        MatrixXd GenerateSquareUVCoord(const Vector3d &start, const Vector3d &u_end, const Vector3d &v_end, int num_u_segments, int num_v_segments);
        MatrixXi GenerateSquareFaceTopo(int num_u_segments, int num_v_segments);
        VectorXd GenerateSquareMass(double density, const VectorXd& x_rest, const MatrixXi &topo);

        VectorXd CalculateArea(const VectorXd& x, const MatrixXi& face_topo);

        std::vector<Matrix2d> CalculateInv(const MatrixXd& uv_coord, const MatrixXi& face_topo);
        std::vector<Matrix6d> CalculatePFPX(const MatrixXd& uv_coord, const MatrixXi& face_topo);
        
    }

    namespace BezierCurve {
        VectorXd GenerateSamplePoints(const VectorXd& control_points, int num_segments);
        SparseMatrixXd GenerateBase(int num_segments);
        VectorXd GenerateShift(int num_segments);
    }

    namespace BezierSurface {
        VectorXd GenerateSamplePoints(const VectorXd& control_points, int num_u_segments, int num_v_segments);
        MatrixXd GenerateUVCoord(const VectorXd &control_points, int num_u_segments, int num_v_segments);
        Vector2d GenerateUVDir(const VectorXd &control_points, const Vector3d &dir);
        SparseMatrixXd GenerateBase(int num_u_segments, int num_v_segments);
        VectorXd GenerateShift(int num_u_segments, int num_v_segments);
    }
}

