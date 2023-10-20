#include "gtest/gtest.h"
#include "Collision/CollisionUtil/CCD/SimpleCCD.h"
#include "Collision/CollisionUtil/CCD/CubicSolver/BisectionCubicSolver.h"
#include "Collision/CollisionUtil/CCD/CubicSolver/CemCubicSolver.hpp"

TEST(CCDTest, SimpleCCDTest) {
	Vector3d x11, x12, x21, x22;
	Vector3d v11, v12, v21, v22;
	x11 << -0.190180, 0.000000, 0.061901;
	x12 << -0.197655, 0.005557, 0.030030;
	x21 << -0.197655, -0.005557, 0.030030;
	x22 << -0.200000, 0.000000, 0.000000;
	v11 << 0, 0, 0;
	v12 << 0, 0, 0;
	v21 << 0, 0, 0;
	v22 << 0, 0, 0;
	SimpleCCD ccd(1e-10, new CemCubicSolver(1e-10));
	EXPECT_EQ(ccd.EdgeEdgeCollision(x11, x12, x21, x22, v11, v12, v21, v22), 2);
}


