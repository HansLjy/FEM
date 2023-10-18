#include "gtest/gtest.h"
#include "Collision/CollisionUtil/CCD/SimpleCCD.h"
#include "Collision/CollisionUtil/CCD/CubicSolver/BisectionCubicSolver.h"

TEST(CCDTest, SimpleCCDTest) {
	Vector3d x11, x12, x21, x22;
	Vector3d v11, v12, v21, v22;
	x11 << 0, 0, 0;
	x12 << 1, 1, 0;
	x21 << 1, 0, 0;
	x22 << 2, 1, 0;
	v11 << 0, 0, 1;
	v12 << 0, 0, 1;
	v21 << 0, 0, 1;
	v22 << 0, 0, 1;
	SimpleCCD ccd(1e-10, new BisectionCubicSolver(1e-10));
	EXPECT_EQ(ccd.EdgeEdgeCollision(x11, x12, x21, x22, v11, v12, v21, v22), 2);
}