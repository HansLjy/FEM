//
// Created by hansljy on 10/28/22.
//

#include "Cloth/Cloth.h"
#include "FiniteDifference.h"
#include "gtest/gtest.h"
#include "spdlog/spdlog.h"

class ClothForTest;
void ClothRandomize(ClothForTest& cloth);

class ClothForTest : public Cloth {
public:
    ClothForTest(double rho, double thickness, double k_stretch, double k_shear, double k_bend,
                 const Vector3d& start, const Vector3d& u_end, const Vector3d& v_end,
                 int num_u_segments, int num_v_segments, double stretch_u, double stretch_v)
                 : Object(Cloth::GeneratePosition(start, u_end, v_end, num_u_segments, num_v_segments)),
                   Cloth(rho, thickness, k_stretch, k_shear, k_bend, k_bend, Vector2d::Random(),
                         start, u_end, v_end,
                         num_u_segments, num_v_segments) {}

    FRIEND_TEST(ClothTest, ClothInitializationTest);
    FRIEND_TEST(ClothTest, ClothEnergyTest);
    friend void ClothRandomize(ClothForTest& cloth);
};

const double rho = 1;
const double k_stretch = 100;
const double k_shear = 1;
const double k_bend = 0.1;
const double stretch_u = 1, stretch_v = 1;
const double thickness = 0.1;
const Vector3d start = (Vector3d() << 0, 0, 0).finished();
const Vector3d u_end = (Vector3d() << 2, 0, 0).finished();
const Vector3d v_end = (Vector3d() << 2, 2, 0).finished();

#include "RandomMatrix.h"

TEST(ClothTest, ClothInitializationTest) {
    const int num_u_segments = 1;
    const int num_v_segments = 1;
    ClothForTest cloth(rho, thickness, k_stretch, k_shear, k_bend, start, u_end, v_end, num_u_segments, num_v_segments, stretch_u, stretch_v);

    EXPECT_EQ(cloth._num_points, 4);
    EXPECT_EQ(cloth._num_triangles, 2);
    EXPECT_EQ(cloth._num_internal_edges, 1);
    EXPECT_EQ(cloth._k_stretch, k_stretch);
    EXPECT_EQ(cloth._k_shear, k_shear);
//    EXPECT_EQ(cloth._k_bend, k_bend);
    // TODO test k_bend
    EXPECT_EQ(cloth._stretch_u, stretch_u);
    EXPECT_EQ(cloth._stretch_v, stretch_v);

    EXPECT_EQ(cloth._mass.size(), 4);
    EXPECT_DOUBLE_EQ(cloth._mass(0), rho * thickness * 2 / 3);
    EXPECT_DOUBLE_EQ(cloth._mass(3), rho * thickness * 2 / 3);
    EXPECT_DOUBLE_EQ(cloth._mass(1), rho * thickness * 4 / 3);
    EXPECT_DOUBLE_EQ(cloth._mass(2), rho * thickness * 4 / 3);

    spdlog::info("UV coordinates:");
    std::cerr << cloth._uv_coord << std::endl;

    EXPECT_EQ(cloth._uv_coord.size(), 8);
    VectorXd uv_coord(8);
    uv_coord <<
        0, 0,
        2, 0,
        2, 2,
        4, 2;
    EXPECT_TRUE((uv_coord - cloth._uv_coord).norm() < 1e-10);

    EXPECT_EQ(cloth._area.size(), 2);
    for (int i = 0; i < 2; i++) {
        EXPECT_DOUBLE_EQ(cloth._area(i), 2);
    }

    MatrixXi topo(2, 3);
    topo << 0, 1, 2,
            1, 3, 2;

    EXPECT_EQ(cloth._topo, topo);

    Matrix2d M1, M2;
    M1 << 2, 2, 0, 2;
    M2 << 2, 0, 2, 2;

    EXPECT_EQ(cloth._inv.size(), 2);
    EXPECT_NEAR((cloth._inv(0) * M1 - Matrix2d::Identity()).norm(), 0, 1e-5);
    EXPECT_NEAR((cloth._inv(1) * M2 - Matrix2d::Identity()).norm(), 0, 1e-5);

    //TODO: Test pFpx

    EXPECT_EQ(cloth._internal_edge.rows(), 1);
    RowVector4i internal_edge;
    internal_edge << 1, 2, 0, 3;
    EXPECT_EQ(internal_edge, cloth._internal_edge.row(0));

    EXPECT_EQ(cloth._internal_edge_length.size(), 1);
    EXPECT_DOUBLE_EQ(cloth._internal_edge_length(0), 2);
}

TEST(ClothTest, ClothEnergyTest) {
    //TODO: Add a specific example

    const int num_u_segments = 10;
    const int num_v_segments = 10;
    ClothForTest cloth(rho, thickness, k_stretch, k_shear, k_bend, start, u_end, v_end, num_u_segments, num_v_segments, stretch_u, stretch_v);

    Matrix3d R = GetRandomRotationMatrix();
    Vector3d b = Vector3d::Random();

    for (int i = 0, j = 0; i < cloth._num_points; i++, j += 3) {
        cloth._x.segment<3>(j) = R * cloth._x.segment<3>(j) + b;
    }

    EXPECT_NEAR(cloth.GetPotential(cloth._x), 0, 1e-10);
}

#include "DerivativeTest.h"

void ClothRandomize(ClothForTest& cloth) {
    cloth._x.setRandom();
}

TEST(ClothTest, ClothEnergyDerivativeTest) {
    const int num_u_segments = 1;
    const int num_v_segments = 1;
    ClothForTest cloth(rho, thickness, 1, 1, 1, start, u_end, v_end, num_u_segments, num_v_segments, stretch_u, stretch_v);

    GenerateDerivatives(cloth, Potential, ClothRandomize, 1e-8, 1e-4)

    PrintGradient()
    PrintHessian()

    EXPECT_NEAR((numeric_gradient - analytic_gradient).norm() / numeric_gradient.size(), 0, 1e-2);
    EXPECT_NEAR((numeric_hessian - analytic_hessian).norm() / numeric_hessian.size(), 0, 1e-2);
}