#pragma once

#include "EigenAll.h"

namespace ReducedDataUtils {
    namespace BezierCurve {
        VectorXd GenerateSamplePoints(const VectorXd& control_points, int num_segments);
        SparseMatrixXd GenerateBase(int num_segments);
        VectorXd GenerateShift(int num_segments);
    }

    namespace BezierSurface {
        VectorXd GenerateSamplePoints(const VectorXd& control_points, int num_u_segments, int num_v_segments);
        VectorXd GenerateUVCoord(const VectorXd &control_points, int num_u_segments, int num_v_segments);
        Vector2d GenerateUVDir(const VectorXd &control_points, const Vector3d &dir);
        SparseMatrixXd GenerateBase(int num_u_segments, int num_v_segments);
        VectorXd GenerateShift(int num_u_segments, int num_v_segments);
    }
};