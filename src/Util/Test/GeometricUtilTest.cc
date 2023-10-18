#include "gtest/gtest.h"
#include "GeometryUtil.hpp"

TEST(GeometricUtilTest, ClosestPointTest) {
	/* point plane case */

	for (int i = 0; i < 100; i++) {
		Vector3d x1 = Vector3d::Random();
		Vector3d x2 = Vector3d::Random();
		Vector3d x3 = Vector3d::Random();
		Vector3d c = Vector3d::Random();

		Vector2d lambda = GeometryUtil::GetPointPlaneClosestPoint(x1, x2, x3, c);
		Vector3d perp = c - (x1 + lambda(0) * (x2 - x1) + lambda(1) * (x3 - x1));
		EXPECT_NEAR(perp.dot(x2 - x1), 0, 1e-10);
		EXPECT_NEAR(perp.dot(x3 - x1), 0, 1e-10);

		Vector3d x11 = Vector3d::Random();
		Vector3d x12 = Vector3d::Random();
		Vector3d x21 = Vector3d::Random();
		Vector3d x22 = Vector3d::Random();

		lambda = GeometryUtil::GetLineLineClosestPoint(x11, x12, x21, x22);
		perp = x11 + lambda(0) * (x12 - x11) - (x21 + lambda(1) * (x22 - x21));
		EXPECT_NEAR(perp.dot(x12 - x11), 0, 1e-10);
		EXPECT_NEAR(perp.dot(x22 - x21), 0, 1e-10);
	}

}