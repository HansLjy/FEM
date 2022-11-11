//
// Created by hansljy on 11/1/22.
//

#include "gtest/gtest.h"
#include "ReducedObject/ReducedBezierSurface.h"
#include "spdlog/spdlog.h"

class ReducedBezierSurfaceForTest : public ReducedBezierSurface {
public:
    ReducedBezierSurfaceForTest(const VectorXd& control_points, double rho, double k_stretch, double k_shear,
                                double k_bend, int num_u_segments, int num_v_segments, double stretch_u, double stretch_v)
                                : ReducedBezierSurface(control_points, rho, k_stretch, k_shear, k_bend, num_u_segments, num_v_segments, stretch_u, stretch_v) {}

    FRIEND_TEST(ReducedObjectTest, ReducedBezierSurfaceInitializationTest);
};

const VectorXd surface_control_points = (VectorXd(27) <<
        0, 0, 20,
        7.5, 0, 20,
        15, 0, 20,
        0, 5, 20,
        7.5, 5, 20,
        15, 5, 20,
        0, 10, 20,
        7.5, 10, 20,
        15, 10, 20
).finished();

const double rho = 1;
const double k_stretch = 1000;
const double k_shear = 1000;
const double k_bend = 1;
const int num_u_segments = 3;
const int num_v_segments = 3;
const double stretch_u = 1;
const double stretch_v = 1;

#include "RandomMatrix.h"

TEST(ReducedObjectTest, ReducedBezierSurfaceInitializationTest) {
    int num_u_segments = 3;
    int num_v_segments = 2;
    int num_sample_points = (num_u_segments + 1) * (num_v_segments + 1);

    Vector3d lower_left = surface_control_points.segment<3>(0);
    VectorXd x_correct(num_sample_points * 3);
    double delta_x = 5;
    double delta_y = 5;
    for (int i = 0; i <= num_u_segments; i++) {
        for (int j = 0; j <= num_v_segments; j++) {
            x_correct.segment<3>((j * (num_u_segments + 1) + i) * 3)
                    << delta_x * i + lower_left(0), delta_y * j + lower_left(1), lower_left(2);
        }
    }
    VectorXd uv_correct(num_sample_points * 2);
    for (int i = 0; i <= num_u_segments; i++) {
        for (int j = 0; j <= num_v_segments; j++) {
            uv_correct.segment<2>((j * (num_u_segments + 1) + i) * 2)
                    << delta_x * i, delta_y * j;
        }
    }

    auto x = ReducedBezierSurfaceForTest::GenerateX(surface_control_points, num_u_segments, num_v_segments);
    spdlog::info("X: ");
    for (int i = 0; i < num_sample_points; i++) {
        std::cout << x.segment<3>(i * 3).transpose() << std::endl;
    }
    EXPECT_NEAR((x_correct - x).norm(), 0, 1e-5);

    auto uv = ReducedBezierSurfaceForTest::GenerateUVCoord(surface_control_points, num_u_segments, num_v_segments);
    spdlog::info("UV: ");
    for (int i = 0; i < (num_u_segments + 1) * (num_v_segments + 1); i++) {
        std::cerr << uv.segment<2>(2 * i).transpose() << std::endl;
    }
    EXPECT_NEAR((uv_correct - uv).norm(), 0, 1e-5);

    Matrix3d R = GetRandomRotationMatrix();
    Vector3d b = Vector3d::Random();

    // Rigid motion won't change uv space
    VectorXd control_points = surface_control_points;
    for (int i = 0; i < 9; i++) {
        control_points.segment<3>(3 * i) = R * control_points.segment<3>(3 * i) + b;
    }
    auto uv_same = ReducedBezierSurfaceForTest::GenerateUVCoord(control_points, num_u_segments, num_v_segments);
    EXPECT_NEAR((uv_correct - uv_same).norm(), 0, 1e-5);
}



void Randomize(ReducedBezierSurfaceForTest& surface) {
    surface.SetCoordinate(Eigen::Vector<double, 27>::Random());
}

#include "DerivativeTest.h"

TEST(ReducedObjectTest, ReducedObjectEnergyDerivativeTest) {
    ReducedBezierSurfaceForTest surface(surface_control_points, rho, k_stretch, k_shear, k_bend, num_u_segments, num_v_segments, stretch_u, stretch_v);
    GenerateDerivativesWithInfo(surface, Energy, Randomize, 1e-8, 1e-4)

    std::cout << "Energy: " << surface.GetEnergy(Matrix3d::Identity(), Vector3d::Zero()) << std::endl;

//    PrintGradient()
    EXPECT_NEAR((numeric_gradient - analytic_gradient).norm() / numeric_gradient.norm(), 0, 1e-2);

//    PrintHessian()
    EXPECT_NEAR((numeric_hessian - analytic_hessian).norm() / numeric_hessian.norm(), 0, 1e-2);
}