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

TEST(ReducedObjectTest, ReducedBezierSurfaceInitializationTest) {
    VectorXd control_points = (VectorXd(27) <<
        0, 0, 20,
        5, 0, 20,
        10, 0, 20,
        0, 5, 20,
        5, 5, 20,
        10, 5, 20,
        0, 10, 20,
        5, 10, 20,
        10, 10, 20
    ).finished();
    VectorXd uv_coord = (VectorXd(18) <<
        0, 0,
        5, 0,
        10, 0,
        0, 5,
        5, 5,
        10, 5,
        0, 10,
        5, 10,
        10, 10
    ).finished();

    const int num_u_segments = 2;
    const int num_v_segments = 2;

    auto x = ReducedBezierSurfaceForTest::GenerateX(control_points, num_u_segments, num_v_segments);
    spdlog::info("X: ");
    for (int i = 0; i < (num_u_segments + 1) * (num_v_segments + 1); i++) {
        std::cout << x.segment<3>(i * 3).transpose() << std::endl;
    }
    EXPECT_NEAR((control_points - x).norm(), 0, 1e-5);

    auto topo = ReducedBezierSurfaceForTest::GenerateTopo(num_u_segments, num_v_segments);
    spdlog::info("Topo: ");
    std::cout << topo << std::endl;

    auto uv = ReducedBezierSurfaceForTest::GenerateUVCoord(control_points, num_u_segments, num_v_segments);
    spdlog::info("UV: ");
    for (int i = 0; i < (num_u_segments + 1) * (num_v_segments + 1); i++) {
        std::cerr << uv.segment<2>(2 * i).transpose() << std::endl;
    }
    EXPECT_NEAR((uv - uv_coord).norm(), 0, 1e-5);
}


const VectorXd surface_control_points = (VectorXd(27) <<
    0, 0, 20,
    5, 0, 20,
    10, 0, 20,
    0, 5, 20,
    5, 5, 20,
    10, 5, 20,
    0, 10, 20,
    5, 10, 20,
    10, 10, 20
).finished();

const double rho = 1;
const double k_stretch = 1000;
const double k_shear = 1000;
const double k_bend = 1;
const int num_u_segments = 3;
const int num_v_segments = 3;
const double stretch_u = 1;
const double stretch_v = 1;

void Randomize(ReducedBezierSurfaceForTest& surface) {
    surface.SetCoordinate(Eigen::Vector<double, 27>::Random());
}

#include "DerivativeTest.h"

TEST(ReducedObjectTest, ReducedObjectEnergyDerivativeTest) {
    ReducedBezierSurfaceForTest surface(surface_control_points, rho, k_stretch, k_shear, k_bend, num_u_segments, num_v_segments, stretch_u, stretch_v);
    GenerateDerivatives(surface, Energy, Randomize, 1e-8, 1e-4)

    std::cout << "Energy: " << surface.GetEnergy() << std::endl;

//    PrintGradient()
    EXPECT_NEAR((numeric_gradient - analytic_gradient).norm() / numeric_gradient.norm(), 0, 1e-2);

//    PrintHessian()
    EXPECT_NEAR((numeric_hessian - analytic_hessian).norm() / numeric_hessian.norm(), 0, 1e-2);
}