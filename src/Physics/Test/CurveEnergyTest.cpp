//
// Created by hansljy on 10/11/22.
//

#include "gtest/gtest.h"
#include "Curve/Curve.h"
#include "spdlog/spdlog.h"
#include "TestUtility/FiniteDifference.h"
#include <functional>
#include <iostream>

class CurveForTest : public Curve {
public:
    CurveForTest(const Vector3d &start, const Vector3d &end, int num_segments, double total_mass, double alpha)
        : Curve(start, end, num_segments, total_mass, alpha) {}

    FRIEND_TEST(CurveTest, CurveInitializationTest);
    FRIEND_TEST(CurveTest, CurveEnergyTest);
    FRIEND_TEST(CurveTest, CurveGradientTest);
    FRIEND_TEST(CurveTest, CurveGravityTest);
    FRIEND_TEST(GravityTest, CurveGravityDerivativeTest);
};



const double eps = 1e-10;

TEST(CurveTest, CurveInitializationTest) {
    Vector3d start, end;
    start << 0, 0, 0;
    end << 0, 0, 1;
    CurveForTest curve(start, end, 2, 3, 0.1);
    EXPECT_EQ(curve._alpha, 0.1);
    EXPECT_EQ(curve._num_points, 3);
    EXPECT_EQ(curve._x_rest.size(), 9);
    EXPECT_EQ(curve._mass.size(), 3);
    EXPECT_EQ(curve._mass_sparse.size(), 9);
    EXPECT_EQ(curve._rest_length.size(), 2);
    EXPECT_EQ(curve._voronoi_length.size(), 3);

    for (int i = 0; i < 3; i++) {
        EXPECT_EQ(curve._mass(i), 1);
    }
    EXPECT_EQ(curve._voronoi_length(0), 0.25);
    EXPECT_EQ(curve._voronoi_length(1), 0.5);
    EXPECT_EQ(curve._voronoi_length(2), 0.25);

    for (int i = 0; i < 2; i++) {
        EXPECT_EQ(curve._rest_length(i), 0.5);
    }

    for (int i = 0; i < 9; i++) {
        EXPECT_EQ(curve._mass_sparse(i), 1);
    }

    EXPECT_EQ(curve._x_rest, curve._x);

    Vector3d middle = (start + end) / 2;
    EXPECT_EQ(Vector3d(curve._x_rest.block<3, 1>(3, 0)), middle);
}

TEST(CurveTest, CurveEnergyTest) {
    Vector3d start, end;
    start << 0, 0, 0;
    end << 0, 0, 1;
    CurveForTest curve(start, end, 100, 3, 0.1);

    EXPECT_EQ(curve.GetEnergy(), 0);    // rest shape

    Matrix3d R;
    Vector3d b, center, axis;

    center.setRandom();
    b.setRandom();
    axis.setRandom();
    R = AngleAxisd(100, axis);

    for (int i = 0; i < 101; i++) {
        Vector3d x = curve._x.block<3, 1>(3 * i, 0);
        curve._x.block<3, 1>(3 * i, 0) = R * (x - center) + b;
    }

    // rigid motion
    EXPECT_LT(curve.GetEnergy(), 1e-10);
    EXPECT_GT(curve.GetEnergy(), -1e-10);

    double length = 1.0 / 100;
    for (int i = 1; i <= 100; i++) {
        Vector3d delta;
        delta.setRandom();
        delta.normalize();
        delta *= length;
        curve._x.block<3, 1>(3 * i, 0) = curve._x.block<3, 1>(3 * (i - 1), 0) + delta;
    }
    EXPECT_GT(curve.GetEnergy(), 0);

    curve._alpha = 0;
    EXPECT_EQ(curve.GetEnergy(), 0);

    CurveForTest simple_curve(start, end, 2, 1, 0.1);
    simple_curve._x.block<3, 1>(3, 0) << 0.5, 0, 0;
    simple_curve._x.block<3, 1>(6, 0) << 0.5, 0.5, 0;

    double k = 2 * tan(EIGEN_PI / 4);
    EXPECT_FLOAT_EQ(0.1 * k * k / 0.5, simple_curve.GetEnergy());
}

TEST(CurveTest, CurveGradientTest) {
    const int num_segments = 10;

    Vector3d start, end;
    start << 0, 0, 0;
    end << 0, 0, 1;
    CurveForTest curve(start, end, num_segments, 3, 0.1);

    auto func = [&curve] (const VectorXd& x) {
        curve._x = x;
        return curve.GetEnergy();
    };

    double length = 1.0 / num_segments;
    VectorXd x(curve._x);
    for (int i = 1; i <= num_segments; i++) {
        Vector3d delta;
        delta.setRandom();
        delta.normalize();
        delta *= length;
        x.block<3, 1>(3 * i, 0) = x.block<3, 1>(3 * (i - 1), 0) + delta;
    }

    curve.GetCoordinate() = x;
    auto numeric_gradient = FiniteDifferential(func, x);
    auto analytic_gradient = curve.GetEnergyGradient();
//    std::cout << "Analytic gradient: \n" << analytic_gradient.transpose() << std::endl;
//    std::cout << "Numeric gradient: \n" << numeric_gradient.transpose() << std::endl;
//    std::cout << "Difference: \n" << (numeric_gradient - analytic_gradient).transpose() << std::endl;
    EXPECT_TRUE((numeric_gradient - analytic_gradient).norm() < 0.1);
}

#include "ExternalForce/Curve/CurveGravity.h"

TEST(CurveTest, CurveGravityTest) {
    Vector3d g;
    double g_norm = 9.8;
    g << 0, 0, -g_norm;
    CurveGravity gravity(g);

    Vector3d start, end;
    start << 0, 0, 1;
    end << 0, 0, -1;
    const int num_segments = 10;
    const double mass = 1;
    CurveForTest curve(start, end, num_segments, mass, 0.1);
    curve.AddExternalForce(gravity);

    EXPECT_NEAR(curve.GetExternalEnergy(), 0, 1e-15);  // neutral

    double length = 1.0 / num_segments;
    for (int i = 1; i <= num_segments; i++) {
        Eigen::Vector2d delta;
        delta.setRandom();
        delta.normalize();
        delta *= length;
        Vector3d delta_3d;
        delta_3d << delta(0), delta(1), 0;
        curve._x.block<3, 1>(3 * i, 0) = curve._x.block<3, 1>(3 * (i - 1), 0) + delta_3d;
    }
    EXPECT_NEAR(curve.GetExternalEnergy(), mass * g_norm * 1, 1e-10);
}

TEST(GravityTest, CurveGravityDerivativeTest) {
    Vector3d start, end;
    start << 0, 0, 0;
    end << 0, 0, 1;
    const int num_segments = 10;
    const double mass = 1;
    CurveForTest curve(start, end, num_segments, mass, 0.1);

    double length = 1.0 / num_segments;
    for (int i = 1; i <= num_segments; i++) {
        Vector3d delta;
        delta.setRandom();
        delta.normalize();
        delta *= length;
        curve._x.block<3, 1>(3 * i, 0) = curve._x.block<3, 1>(3 * (i - 1), 0) + delta;
    }

    VectorXd x_backup = curve._x;
    auto func = [&curve] (const VectorXd& x) {
        curve._x = x;
        return curve.GetExternalEnergy();
    };

    auto numeric_gradient = FiniteDifferential(func, x_backup);

    curve._x = x_backup;
    auto analytic_gradient = curve.GetExternalEnergyGradient();

    EXPECT_NEAR((numeric_gradient - analytic_gradient).norm() / numeric_gradient.size(), 0, 1e-5);

    auto numeric_hessian = FiniteDifferential2(func, x_backup);
    curve._x = x_backup;
    SparseMatrixXd analytic_hessian(curve.GetDOF(), curve.GetDOF());
    COO coo;
    curve.GetExternalEnergyHessian(coo, 0, 0);
    analytic_hessian.setFromTriplets(coo.begin(), coo.end());
    EXPECT_NEAR((numeric_hessian - analytic_hessian).norm() / numeric_hessian.size(), 0, 1e-5);
}