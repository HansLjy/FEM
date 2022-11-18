//
// Created by hansljy on 10/11/22.
//

#include "gtest/gtest.h"
#include "Curve/InextensibleCurve.h"
#include "Curve/ExtensibleCurve.h"
#include "spdlog/spdlog.h"
#include "TestUtility/FiniteDifference.h"
#include <functional>
#include <iostream>


class InextensibleCurveForTest;
void InextensibleRandomize(InextensibleCurveForTest& curve);

class InextensibleCurveForTest : public InextensibleCurve {
public:
    InextensibleCurveForTest(const Vector3d &start, const Vector3d &end, int num_segments, double rho, double alpha)
        : Object(Curve::GetX(start, end, num_segments)), InextensibleCurve(rho, alpha, alpha, start, end, num_segments) {}

    FRIEND_TEST(CurveTest, CurveInitializationTest);
    FRIEND_TEST(CurveTest, CurveEnergyTest);
    FRIEND_TEST(CurveTest, CurveGradientTest);
    FRIEND_TEST(CurveTest, CurveGravityTest);
    FRIEND_TEST(GravityTest, CurveGravityDerivativeTest);

    friend void InextensibleRandomize(InextensibleCurveForTest& curve);
};

class ExtensibleCurveForTest;
void ExtensibleRandomize(ExtensibleCurveForTest& curve);

class ExtensibleCurveForTest : public ExtensibleCurve {
public:
    ExtensibleCurveForTest(const Vector3d &start, const Vector3d &end, int num_segments, double rho, double alpha)
        : Object(Curve::GetX(start, end, num_segments)), ExtensibleCurve(rho, alpha, alpha, start, end, num_segments) {}

    FRIEND_TEST(ExtensibleCurveTest, EnergyTest);
    FRIEND_TEST(ExtensibleCurveTest, EnergyGradientTest);

    friend void ExtensibleRandomize(ExtensibleCurveForTest& curve);
};

const double eps = 1e-10;

const double alpha = 0.1;
const double rho = 3;
const Vector3d start((Vector3d() << 0, 0, 0).finished());
const Vector3d end((Vector3d() << 0, 0, 1).finished());
const int num_segments = 3;
const double length = (start - end).norm() / num_segments;

TEST(CurveTest, CurveInitializationTest) {
    InextensibleCurveForTest curve(start, end, num_segments, rho, alpha);
    EXPECT_DOUBLE_EQ(curve._alpha(1), alpha);
    EXPECT_DOUBLE_EQ(curve._num_points, num_segments + 1);
    EXPECT_DOUBLE_EQ(curve._x_rest.size(), 3 * (num_segments + 1));
    EXPECT_DOUBLE_EQ(curve._mass.size(), num_segments + 1);
    EXPECT_DOUBLE_EQ(curve._rest_length.size(), num_segments);
    EXPECT_DOUBLE_EQ(curve._voronoi_length.size(), num_segments + 1);

    for (int i = 0; i <= num_segments; i++) {
        EXPECT_DOUBLE_EQ(curve._mass(i), rho * curve._voronoi_length(i));
    }

    EXPECT_DOUBLE_EQ(curve._voronoi_length(0), length / 2);
    EXPECT_DOUBLE_EQ(curve._voronoi_length(num_segments), length / 2);
    for (int i = 1; i < num_segments; i++) {
        EXPECT_DOUBLE_EQ(curve._voronoi_length(i), length);
    }

    for (int i = 0; i < num_segments; i++) {
        EXPECT_DOUBLE_EQ(curve._rest_length(i), length);
    }

    EXPECT_EQ(curve._x_rest, curve._x);
}

void InextensibleRandomize(InextensibleCurveForTest& curve) {
    for (int i = 1; i <= num_segments; i++) {
        Vector3d delta;
        delta.setRandom();
        delta.normalize();
        delta *= length;
        curve._x.block<3, 1>(3 * i, 0) = curve.GetCoordinate().block<3, 1>(3 * (i - 1), 0) + delta;
    }
}

TEST(CurveTest, CurveEnergyTest) {
    InextensibleCurveForTest curve(start, end, num_segments, rho, alpha);

    EXPECT_EQ(curve.GetPotential(curve._x), 0);    // rest shape

    Matrix3d R;
    Vector3d b, center, axis;

    center.setRandom();
    b.setRandom();
    axis.setRandom();
    R = AngleAxisd(num_segments, axis);

    for (int i = 0; i <= num_segments; i++) {
        Vector3d x = curve._x.block<3, 1>(3 * i, 0);
        curve._x.block<3, 1>(3 * i, 0) = R * (x - center) + b;
    }

    // rigid motion
    EXPECT_LT(curve.GetPotential(curve._x), 1e-10);
    EXPECT_GT(curve.GetPotential(curve._x), -1e-10);

    InextensibleRandomize(curve);
    EXPECT_GT(curve.GetPotential(curve._x), 0);

    for (int i = 1; i < num_segments; i++) {
        curve._alpha(i) = 0;
    }
    EXPECT_EQ(curve.GetPotential(curve._x), 0);

    InextensibleCurveForTest simple_curve(start, end, 2, 1, 0.1);
    simple_curve._x.block<3, 1>(3, 0) << 0.5, 0, 0;
    simple_curve._x.block<3, 1>(6, 0) << 0.5, 0.5, 0;

    double k = 2 * tan(EIGEN_PI / 4);
    EXPECT_FLOAT_EQ(0.1 * k * k / 0.5, simple_curve.GetPotential(simple_curve._x));
}

#include "DerivativeTest.h"

TEST(CurveTest, CurveGradientTest) {
    InextensibleCurveForTest curve(start, end, num_segments, rho, alpha);

    GenerateDerivatives(curve, Potential, InextensibleRandomize, 1e-8, 1e-4)

    std::cout << "Analytic gradient: \n" << analytic_gradient.transpose() << std::endl;
    std::cout << "Numeric gradient: \n" << numeric_gradient.transpose() << std::endl;
    std::cout << "Difference: \n" << (numeric_gradient - analytic_gradient).transpose() << std::endl;
    EXPECT_NEAR((numeric_gradient - analytic_gradient).norm() / numeric_gradient.size(), 0, 1e-2);

    std::cerr << "Numeric Hessian: \n" << numeric_hessian << std::endl;
    std::cerr << "Analytic Hessian: \n" << analytic_hessian.toDense() << std::endl;
    std::cerr << "Hessian Difference: \n" << numeric_hessian - analytic_hessian.toDense() << std::endl;
    EXPECT_NEAR((numeric_hessian - analytic_hessian).norm() / numeric_hessian.size(), 0, 0.1);
}

#include "ExternalForce/SampledObject/SampledObjectGravity.h"

#include <random>

void ExtensibleRandomize(ExtensibleCurveForTest& curve) {
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.8, 1.2);
    for (int i = 1, j = 3; i <= num_segments; i++, j += 3) {
        double cur_length = distribution(generator) * length;
        Vector3d delta;
        delta.setRandom().normalize();
        delta *= cur_length;
        curve._x.segment<3>(j) = curve.GetCoordinate().segment<3>(j - 3) + delta;
    }
}

TEST(ExtensibleCurveTest, EnergyGradientTest) {
    ExtensibleCurveForTest curve(start, end, num_segments, rho, alpha);

    GenerateDerivatives(curve, Potential, ExtensibleRandomize, 1e-8, 1e-4)

    std::cerr << "Analytic derivative: " << analytic_gradient.transpose() << std::endl;
    std::cerr << "Numeric derivative: " << numeric_gradient.transpose() << std::endl;
    std::cerr << "Difference: " << (analytic_gradient - numeric_gradient).transpose() << std::endl;

    EXPECT_NEAR((analytic_gradient - numeric_gradient).norm() / curve.GetDOF(), 0, 1e-4);

    std::cerr << "Analytic hessian: \n" << analytic_hessian.toDense() << std::endl;
    std::cerr << "Numeric hessian: \n" << numeric_hessian << std::endl;
    std::cerr << "Hessian difference: \n" << numeric_hessian - analytic_hessian.toDense() << std::endl;

    EXPECT_TRUE((numeric_hessian - analytic_hessian).norm() / (curve.GetDOF() * curve.GetDOF()) < 1);
}