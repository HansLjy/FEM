//
// Created by hansljy on 11/8/22.
//


#include "RandomMatrix.h"
//#include "Tree/TreeTrunk.h"
#include <gtest/gtest.h>

//TEST(TreeTest, TreeTrunkFrameValueTest) {
//    const Vector12d control_points = (Vector12d() <<
//            0, 1, 0,
//            0, 1, 4.0 / 3,
//            0, -1, 4.0 / 3,
//            0, -1, 0
//    ).finished();
//    const Vector3d root = (Vector3d() <<
//            0, 1, -1
//    ).finished();
//
//    TreeTrunk tree_trunk(2, 0, 0, 0, control_points, root);
//
//    const double position = 0.75;    // second segment
//    const Matrix3d rest_rotation = Matrix3d::Identity();
//    const std::vector<double> position_child {position};
//    const std::vector<Matrix3d> rest_rotation_child {rest_rotation};
//
//    Vector3d x_parent = Vector3d::Zero();
//    Vector3d v_parent = Vector3d::Zero();
//    Vector3d a_parent = Vector3d::Zero();
//    Vector3d omega_parent = Vector3d::Zero();
//    double alpha_parent_parallel = rand();
//    Vector3d alpha_parent = omega_parent.normalized() * alpha_parent_parallel;
//    Matrix3d rotation_parent = Matrix3d::Identity();
//
//    tree_trunk.SetFrameInfo(x_parent, v_parent, a_parent, omega_parent, alpha_parent, rotation_parent);
//
//    std::vector<Vector3d> x_child, v_child, a_child, omega_child, alpha_child;
//    std::vector<Matrix3d> rotation_child;
//
//    Vector12d a = Vector12d::Zero();
//
//    tree_trunk.CalculateChildrenFrame(a, position_child, rest_rotation_child,
//                                      x_child, v_child, a_child,
//                                      omega_child, alpha_child, rotation_child);
//
//    Vector3d expected_x_child = (Vector3d() << 0, -0.5, 0.5).finished();
//    Matrix3d expected_rotation_child = (Matrix3d() <<
//        1, 0, 0,
//        0, -sqrt(2) / 2, -sqrt(2) / 2,
//        0, sqrt(2) / 2, -sqrt(2) / 2
//    ).finished();
//    EXPECT_NEAR((expected_x_child - x_child[0]).norm(), 0, 1e-5);
//    EXPECT_NEAR((expected_rotation_child - rotation_child[0]).norm(), 0, 1e-5);
//}
//
//// Test the calculation of the dynamics of frame (velocity, angular velocity, etc.)
//TEST(TreeTest, TreeTrunkFrameTest) {
//    const Vector12d control_points = (Vector12d() <<
//        0, 0, 0,
//        0, 0, 3,
//        0, 0, 6,
//        0, 0, 9
//    ).finished();
//    const Vector3d root = (Vector3d() <<
//        0, 0, -1
//    ).finished();
//
//    TreeTrunk tree_trunk(1, 0, 0, 0, control_points, root);
//    const double position = 0.7;    // second segment
//    const Matrix3d rest_rotation = Matrix3d::Identity();
//    const std::vector<double> position_child {position};
//    const std::vector<Matrix3d> rest_rotation_child {rest_rotation};
//
//    double step = 1e-5;
//
//    Vector12d x_test = control_points;
//    x_test.segment<9>(3).setRandom();
////    Vector12d v_test = Vector12d::Random() * 10;
//    Vector12d v_test = Vector12d::Zero();
////    Vector12d a_test = Vector12d::Random() * 10;
//    Vector12d a_test = Vector12d::Zero();
//
//    tree_trunk.SetCoordinate(x_test);
//    tree_trunk.SetVelocity(v_test);
//
//    const Vector3d x_parent = (Vector3d() << 1, 1, 1).finished();
//    const Vector3d v_parent = Vector3d::Random();
//    const Vector3d a_parent = Vector3d::Random();
//    const Vector3d omega_parent = Vector3d::Random();
//    const double alpha_parent_parallel = rand() % 100 / 100.0;
//    const Vector3d alpha_parent = omega_parent.normalized() * alpha_parent_parallel;
//    const Matrix3d rotation_parent = GetRandomRotationMatrix();
////    Vector3d x_parent = Vector3d::Zero();
////    Vector3d v_parent = Vector3d::Zero();
////    Vector3d a_parent = Vector3d::Zero();
////    Vector3d omega_parent = Vector3d::Zero();
////    double alpha_parent_parallel = 0;
////    Vector3d alpha_parent = Vector3d::Zero();
////    Matrix3d rotation_parent = Matrix3d::Identity();
//
//    std::vector<Vector3d> x_child, v_child, a_child, omega_child, alpha_child;
//    std::vector<Matrix3d> rotation_child;
//
//
//    // calculate the numeric result
//    const int mid = 2;
//    for (int i = -1; i <= 1; i++) {
//        const double h = i * step;
//        Vector12d x = x_test + h * v_test + 0.5 * h * h * a_test;
//        Vector12d v = v_test + h * a_test;
//        Vector12d a = a_test;
//
//        tree_trunk.SetCoordinate(x);
//        tree_trunk.SetVelocity(v);
//
//        Vector3d x_parent_cur = x_parent + h * v_parent + 0.5 * h * h * a_parent;
//        Vector3d v_parent_cur = v_parent + h * a_parent;
//        Vector3d a_parent_cur = a_parent;
//
//        Vector3d omega_parent_cur = omega_parent + h * alpha_parent;
//        Vector3d alpha_parent_cur = alpha_parent;
//        Matrix3d rotation_parent_cur = Matrix3d(AngleAxisd(
//            omega_parent.norm() * h + 0.5 * h * h * alpha_parent_parallel,
//            omega_parent.normalized()
//        )) * rotation_parent;
//
//        tree_trunk.SetFrameInfo(
//            x_parent_cur, v_parent_cur, a_parent_cur,
//            omega_parent_cur, alpha_parent_cur, rotation_parent_cur
//        );
//
//        std::vector<Vector3d> x_child_cur, v_child_cur, a_child_cur, omega_child_cur, alpha_child_cur;
//        std::vector<Matrix3d> rotation_child_cur;
//
//        tree_trunk.CalculateChildrenFrame(
//            a, position_child, rest_rotation_child,
//            x_child_cur, v_child_cur, a_child_cur, omega_child_cur, alpha_child_cur, rotation_child_cur
//        );
//
//        #define MERGE(A) A##_child.insert(A##_child.end(), A##_child_cur.begin(), A##_child_cur.end())
//
//        MERGE(x);
//        MERGE(v);
//        MERGE(a);
//        MERGE(omega);
//        MERGE(alpha);
//        MERGE(rotation);
//    }
//
//    std::cerr << x_child[0].transpose() << std::endl
//              << x_child[1].transpose() << std::endl
//              << x_child[2].transpose() << std::endl;
//
//    std::cerr << "Numeric velocity: " << (x_child[2] - x_child[0]).transpose() / (2 * step) << std::endl;
//    std::cerr << "Analytic velocity: " << v_child[1].transpose() << std::endl;
//    EXPECT_NEAR(((x_child[2] - x_child[0]) / (2 * step) - v_child[1]).norm(), 0, 1e-5);
//
//    std::cerr << "Numeric acceleration: " << (v_child[2] - v_child[0]).transpose() / (2 * step) << std::endl;
//    std::cerr << "Analytic acceleration: " << a_child[1].transpose() << std::endl;
//    EXPECT_NEAR(((v_child[2] - v_child[0]) / (2 * step) - a_child[1]).norm(), 0, 1e-5);
//
//    std::cerr << "Numeric angular velocity: \n" << (rotation_child[2] - rotation_child[0]) / (2 * step) * rotation_child[1].transpose() << std::endl;
//    std::cerr << "Analytic angular velocity: \n" << HatMatrix(omega_child[1]) << std::endl;
//    EXPECT_NEAR(((rotation_child[2] - rotation_child[0]) / (2 * step) * rotation_child[1].transpose() - HatMatrix(omega_child[1])).norm(), 0, 1e-5);
//
//    std::cerr << "Numeric angular acceleration: \n" << ((omega_child[2] - omega_child[0]) / (2 * step)).transpose() << std::endl;
//    std::cerr << "Analytic angular acceleration: \n" << alpha_child[1].transpose() << std::endl;
//    EXPECT_NEAR(((omega_child[2] - omega_child[0]) / (2 * step) - alpha_child[1]).norm(), 0, 1e-5);
//}