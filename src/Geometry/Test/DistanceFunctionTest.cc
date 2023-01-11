#include "Collision/CollisionUtility.h"
#include "gtest/gtest.h"

TEST(DistanceFunctionTest, VFDistanceTest) {
	Vector3d vertex, face1, face2, face3;
	face1 << 0, 0, 0;
	face2 << 1, 0, 0;
	face3 << 0, 1, 0;

	vertex << 0.5, -1, 1;
	EXPECT_DOUBLE_EQ(GetVFDistance(vertex, face1, face2, face3), sqrt(2));

	vertex << -1, 0.5, -1;
	EXPECT_DOUBLE_EQ(GetVFDistance(vertex, face1, face2, face3), sqrt(2));

	vertex << 1, 1, 1;
	EXPECT_DOUBLE_EQ(GetVFDistance(vertex, face1, face2, face3), sqrt(1.5));

	vertex << 3, -1, -1;
	EXPECT_DOUBLE_EQ(GetVFDistance(vertex, face1, face2, face3), sqrt(6));

	vertex << -1, 3, 1;
	EXPECT_DOUBLE_EQ(GetVFDistance(vertex, face1, face2, face3), sqrt(6));

	vertex << -1, -1, 0;
	EXPECT_DOUBLE_EQ(GetVFDistance(vertex, face1, face2, face3), sqrt(2));
}

TEST(DistanceFunctionTest, EEDistanceTest) {
	Vector3d edge11, edge12, edge21, edge22;
	edge11 << 0, 0, 0;
	edge12 << 0, 0, 1;

	// line-line
	edge21 << 1, 0, 0.5;
	edge22 << 0, 1, 0.5;
	EXPECT_DOUBLE_EQ(GetEEDistance(edge11, edge12, edge21, edge22), sqrt(0.5));

	// vertex-line
	edge21 << 1, 0, 1.5;
	edge22 << 0, 1, 1.5;
	EXPECT_DOUBLE_EQ(GetEEDistance(edge11, edge12, edge21, edge22), sqrt(0.75));

	edge21 << 1, 0, -0.5;
	edge22 << 0, 1, -0.5;
	EXPECT_DOUBLE_EQ(GetEEDistance(edge11, edge12, edge21, edge22), sqrt(0.75));

	edge21 << 0.5, -1.5, 0.5;
	edge22 << 0.5, -0.5, 0.5;
	EXPECT_DOUBLE_EQ(GetEEDistance(edge11, edge12, edge21, edge22), sqrt(0.5));

	edge21 << 0.5, 0.5, 0.5;
	edge22 << 0.5, 1.5, 0.5;
	EXPECT_DOUBLE_EQ(GetEEDistance(edge11, edge12, edge21, edge22), sqrt(0.5));

	// vertex-vertex

	edge21 << 0, 0.5, 1.5;
	edge22 << 0, 1.5, 2.5;
	EXPECT_DOUBLE_EQ(GetEEDistance(edge11, edge12, edge21, edge22), sqrt(0.5));

	edge22 << 0, 0.5, 1.5;
	edge21 << 0, 1.5, 2.5;
	EXPECT_DOUBLE_EQ(GetEEDistance(edge11, edge12, edge21, edge22), sqrt(0.5));

	edge21 << 0, 0.5, -0.5;
	edge22 << 0, 1.5, -1.5;
	EXPECT_DOUBLE_EQ(GetEEDistance(edge11, edge12, edge21, edge22), sqrt(0.5));

	edge22 << 0, 0.5, -0.5;
	edge21 << 0, 1.5, -1.5;
	EXPECT_DOUBLE_EQ(GetEEDistance(edge11, edge12, edge21, edge22), sqrt(0.5));
}

#include "FiniteDifference.h"

#define DEFINE_WRAPPER_12(TYPE, NAME, VALUE) \
	TYPE NAME##Distance##VALUE##Wrapper(const VectorXd& x) {\
		return Get##NAME##Distance##VALUE(x.segment<3>(0), x.segment<3>(3), x.segment<3>(6), x.segment<3>(9));\
	}

#define DEFINE_WRAPPER_9(TYPE, NAME, VALUE) \
	TYPE NAME##Distance##VALUE##Wrapper(const VectorXd& x) {\
		return Get##NAME##Distance##VALUE(x.segment<3>(0), x.segment<3>(3), x.segment<3>(6));\
	}

#define DEFINE_WRAPPER_6(TYPE, NAME, VALUE) \
	TYPE NAME##Distance##VALUE##Wrapper(const VectorXd& x) {\
		return Get##NAME##Distance##VALUE(x.segment<3>(0), x.segment<3>(3));\
	}


DEFINE_WRAPPER_12(double, VF, )
DEFINE_WRAPPER_12(Vector12d, VF, Gradient)
DEFINE_WRAPPER_12(Matrix12d, VF, Hessian)

DEFINE_WRAPPER_12(double, EE, )
DEFINE_WRAPPER_12(Vector12d, EE, Gradient)
DEFINE_WRAPPER_12(Matrix12d, EE, Hessian)

DEFINE_WRAPPER_12(double, VP, )
DEFINE_WRAPPER_12(Vector12d, VP, Gradient)
DEFINE_WRAPPER_12(Matrix12d, VP, Hessian)

DEFINE_WRAPPER_12(double, LL, )
DEFINE_WRAPPER_12(Vector12d, LL, Gradient)
DEFINE_WRAPPER_12(Matrix12d, LL, Hessian)

DEFINE_WRAPPER_9(double, VL, )
DEFINE_WRAPPER_9(Vector9d, VL, Gradient)
DEFINE_WRAPPER_9(Matrix9d, VL, Hessian)

DEFINE_WRAPPER_6(double, VV, )
DEFINE_WRAPPER_6(Vector6d, VV, Gradient)
DEFINE_WRAPPER_6(Matrix6d, VV, Hessian)

#define TEST_DERIVATIVE(SIZE, NAME) \
	Vector##SIZE##d x = Vector##SIZE##d::Random(); \
	VectorXd numeric_gradient = FiniteDifferential(NAME##DistanceWrapper, x, gradient_step);\
	Vector##SIZE##d analytic_gradient = NAME##DistanceGradientWrapper(x);\
	EXPECT_NEAR((numeric_gradient - analytic_gradient).norm() / numeric_gradient.size(), 0, 1e-5); \
	\
	MatrixXd numeric_hessian = FiniteDifferential2(NAME##DistanceWrapper, x, hessian_step);\
	Matrix##SIZE##d analytic_hessian = NAME##DistanceHessianWrapper(x);\
	EXPECT_NEAR((numeric_hessian - analytic_hessian).norm() / numeric_hessian.size(), 0, 1e-5);

#define PRINT_GRADIENT \
	std::cerr << "Numeric gradient:\n" << numeric_gradient.transpose() << std::endl; \
	std::cerr << "Analytic gradient:\n" << analytic_gradient.transpose() << std::endl;\
	std::cerr << "Gradient difference:\n" << (numeric_gradient - analytic_gradient).transpose() << std::endl;

#define PRINT_HESSIAN \
	std::cerr << "Numeric hessian:\n" << numeric_hessian << std::endl; \
	std::cerr << "Analytic hessian:\n" << analytic_hessian << std::endl;\
	std::cerr << "Hessian difference:\n" << (numeric_hessian - analytic_hessian) << std::endl;


const double gradient_step = 1e-10;
const double hessian_step = 1e-5;

TEST(DistanceFunctionTest, VPDistanceDerivativeTest) {
	TEST_DERIVATIVE(12, VP)
}

TEST(DistanceFunctionTest, LLDistanceDerivativeTest) {
	TEST_DERIVATIVE(12, LL)
}

TEST(DistanceFunctionTest, VLDistanceDerivativeTest) {
	TEST_DERIVATIVE(9, VL)
}

TEST(DistanceFunctionTest, VVDistanceDerivativeTest) {
	TEST_DERIVATIVE(6, VV)
}