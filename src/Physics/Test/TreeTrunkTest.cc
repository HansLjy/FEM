//
// Created by hansljy on 11/12/22.
//

#include "gtest/gtest.h"
#include "Object/TreeTrunk.h"

class TreeTrunkForTest;
void TreeTrunkRandomize(TreeTrunkForTest& tree_trunk);

class TreeTrunkForTest : public TreeTrunk {
public:
    TreeTrunkForTest(double rho, double alpha, const Vector3d& start, const Vector3d& end, int num_segments, const Vector3d& root)
        : TreeTrunkForTest(rho, alpha, GenerateX(start, end, num_segments), root) {}

    TreeTrunkForTest(double rho, double youngs_module, const VectorXd &x, const Vector3d& root)
        : TreeTrunk(false, rho, youngs_module, 0.1, 0.1, x, root) {}

    static VectorXd GenerateX(const Vector3d& start, const Vector3d& end, int num_segments) {
        VectorXd x(3 * (num_segments + 1));
        Vector3d delta = (end - start) / num_segments;
        Vector3d current = start;
        for (int i = 0, j = 0; i <= num_segments; i++, j += 3, current += delta) {
            x.segment<3>(j) = current;
        }
        return x;
    }

    friend void TreeTrunkRandomize(TreeTrunkForTest& tree_trunk);
};

const double rho_global = 1;
const double youngs_module = 1;
const Vector3d start_global = (Vector3d() << 0, 0, 0).finished();
const Vector3d end_global = (Vector3d() << 0, 0, 10).finished();
const int segments_global = 2;
const Vector3d root_global = (Vector3d() << 0, 0, -1).finished();

#include "DerivativeTest.h"

void TreeTrunkRandomize(TreeTrunkForTest& tree_trunk) {
    tree_trunk._x.setRandom();
}

TEST(TreeTrunkTest, EnergyGradientTest) {
    TreeTrunkForTest tree_trunk(rho_global, youngs_module, start_global, end_global, segments_global, root_global);
    GenerateDerivatives(tree_trunk, Potential, TreeTrunkRandomize, 1e-8, 1e-4)

//    PrintGradient()
    EXPECT_NEAR((numeric_gradient - analytic_gradient).norm() / numeric_gradient.size(), 0, 1e-2);

//    PrintHessian()
    EXPECT_NEAR((numeric_hessian - analytic_hessian).norm() / numeric_hessian.size(), 0, 1e-2);
}

#include "Object/ReducedTreeTrunk.h"

class ReducedTreeTrunkForTest;
void ReducedTreeTrunkRandomize(ReducedTreeTrunkForTest& tree_trunk);

class ReducedTreeTrunkForTest : public ReducedTreeTrunk {
public:
    ReducedTreeTrunkForTest(int num_segments, double rho, double youngs_module, const Vector3d& root, const VectorXd &control_points)
            : ReducedTreeTrunk(false, num_segments, rho, youngs_module, 0.1, 0.1, root, control_points) {}

    friend void ReducedTreeTrunkRandomize(ReducedTreeTrunkForTest& tree_trunk);
};

void ReducedTreeTrunkRandomize(ReducedTreeTrunkForTest& tree_trunk) {
    VectorXd x(tree_trunk._x);
    x.setRandom();
    tree_trunk.SetCoordinate(x);
}

const Vector12d control_points_global = (Vector12d() <<
    0, 0, 0,
    0, 0, 3,
    0, 0, 6,
    0, 0, 9
).finished();

TEST(TreeTrunkTest, ReducedEnergyTest) {
    ReducedTreeTrunkForTest reduced_tree_trunk(10, rho_global, youngs_module, root_global, control_points_global);

    ReducedTreeTrunkRandomize(reduced_tree_trunk);
//    std::cerr << "Energy: " << reduced_tree_trunk.GetEnergy(Matrix3d::Identity(), Vector3d::Zero());
//    std::cerr << "Gradient: \n" << reduced_tree_trunk.GetEnergyGradient(Matrix3d::Identity(), Vector3d::Zero()).transpose() << std::endl;
//    std::cerr << "Hessian: \n" << reduced_tree_trunk.GetEnergyHessian(Matrix3d::Identity(), Vector3d::Zero()) << std::endl;

    GenerateDerivatives(reduced_tree_trunk, Potential, ReducedTreeTrunkRandomize, 1e-8, 1e-4)

//    PrintGradient()
    EXPECT_NEAR((numeric_gradient - analytic_gradient).norm() / numeric_gradient.size(), 0, 1e-2);

//    PrintHessian()
    EXPECT_NEAR((numeric_hessian - analytic_hessian).norm() / numeric_hessian.size(), 0, 1e-2);

}