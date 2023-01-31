#include "Object/AffineDecomposedTree.h"
#include "gtest/gtest.h"
#include "FiniteDifference.h"
#include "DerivativeTest.h"

#include <functional>
#include <fstream>

class FriendlyAffineTreeTrunk : public AffineDecomposedTreeTrunk {
public:
	FriendlyAffineTreeTrunk(const json& config) : AffineDecomposedTreeTrunk(config) {}

	FRIEND_TEST(AffineDecomposedObjectTest, InitializeTest);
};

TEST(AffineDecomposedObjectTest, InitializeTest) {
	json config;
	std::fstream config_file(PHYSICS_TEST_CONFIG_PATH "/AffineTreeTrunkConfig.json");
	config_file >> config;

	FriendlyAffineTreeTrunk treetrunk(config);
	treetrunk.Initialize();
}

TEST(AffineDecomposedObjectTest, BishopTest) {
	json config;
	std::fstream config_file(PHYSICS_TEST_CONFIG_PATH "/AffineTreeTrunkConfig.json");
	config_file >> config;

	FriendlyAffineTreeTrunk treetrunk(config);

	std::function<double(const VectorXd&)> func[3][3];
	std::function<VectorXd(const VectorXd&)> grad[3][3];
	std::function<MatrixXd(const VectorXd&)> hess[3][3];

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			int index = 3 * j + i;
			func[i][j] = [i, j, &treetrunk](const VectorXd& x) -> double {
				std::vector<Matrix3d> rotations;
				std::vector<MatrixXd> rotation_gradient, rotation_hessian;
				treetrunk.CalculateRigidRotationInfos(AffineDecomposedObject::CalculateLevel::kValue, x, rotations, rotation_gradient, rotation_hessian);
				return rotations[0](i, j);
			};
			grad[i][j] = [index, &treetrunk](const VectorXd& x) -> VectorXd {
				std::vector<Matrix3d> rotations;
				std::vector<MatrixXd> rotation_gradient, rotation_hessian;
				treetrunk.CalculateRigidRotationInfos(AffineDecomposedObject::CalculateLevel::kGradient, x, rotations, rotation_gradient, rotation_hessian);
				return rotation_gradient[0].col(index);
			};
			hess[i][j] = [index, &treetrunk](const VectorXd& x) -> MatrixXd {
				std::vector<Matrix3d> rotations;
				std::vector<MatrixXd> rotation_gradient, rotation_hessian;
				treetrunk.CalculateRigidRotationInfos(AffineDecomposedObject::CalculateLevel::kHessian, x, rotations, rotation_gradient, rotation_hessian);
				return rotation_hessian[0].middleCols(9 * index, 9);
			};
		}
	}

	VectorXd x = VectorXd::Random(9);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			auto numeric_gradient = FiniteDifferential(func[i][j], x, 1e-8);
			auto analytic_gradient = grad[i][j](x);
			EXPECT_NEAR((numeric_gradient - analytic_gradient).norm() / numeric_gradient.size(), 0, 1e-5);

			auto numeric_hessian = FiniteDifferential2(func[i][j], x, 1e-4);
			auto analytic_hessian = hess[i][j](x);
			EXPECT_NEAR((numeric_hessian - analytic_hessian).norm() / numeric_hessian.size(), 0, 1e-3);
		}
	}
}

TEST(AffineDecomposedObjectTest, EnergyTest) {
	json config;
	std::fstream config_file(PHYSICS_TEST_CONFIG_PATH "/AffineTreeTrunkConfig.json");
	config_file >> config;

	FriendlyAffineTreeTrunk treetrunk(config);

	const auto func = [&treetrunk](const VectorXd& x) -> double {
		return treetrunk.GetPotential(x);
	};

	VectorXd x = VectorXd::Random(18);

	auto numeric_gradient = FiniteDifferential(func, x, 1e-8);
	auto analytic_gradient = treetrunk.GetPotentialGradient(x);
	
	EXPECT_NEAR((numeric_gradient - analytic_gradient).norm() / numeric_gradient.size(), 0, 1e-5);

	// PrintGradient()

	auto numeric_hessian = FiniteDifferential2(func, x, 1e-4);
	COO coo;
	treetrunk.GetPotentialHessian(x, coo, 0, 0);
	SparseMatrixXd analytic_hessian(18, 18);
	analytic_hessian.setFromTriplets(coo.begin(), coo.end());
	EXPECT_NEAR((numeric_hessian - analytic_hessian).norm() / numeric_hessian.size(), 0, 1e-3);

	// PrintHessian()
}