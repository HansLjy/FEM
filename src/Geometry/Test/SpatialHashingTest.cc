#include "Collision/SpatialHashing/SpatialHashing.hpp"
#include "gtest/gtest.h"
#include "spdlog/spdlog.h"

bool IntervalIntersect(double a1, double b1, double a2, double b2) {
	return (a1 <= a2 && a2 <= b1) || (a1 <= b2 && b2 <= b1);
}

bool BBIntersect(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) {
	Vector3d bb_min1 = edge11.cwiseMin(edge12);
	Vector3d bb_max1 = edge11.cwiseMax(edge12);
	Vector3d bb_min2 = edge21.cwiseMin(edge22);
	Vector3d bb_max2 = edge21.cwiseMax(edge22);

	for (int i = 0; i < 3; i++)	{
		if (!IntervalIntersect(bb_min1[i], bb_max1[i], bb_min2[i], bb_max2[i])) {
			return false;
		}
	}
	return true;
}

SpatialHashing<int> hash_table(0.1, 4096);

TEST(SpatialHashingTest, Range) {
	int N = 20;
	Vector3d edge[N][2];
	for (int i = 0; i < N; i++) {
		edge[i][0] = Vector3d::Random();
		edge[i][1] = Vector3d::Random();
	}
	for (int i = 0; i < N; i++) {
		Vector3d bb_min = edge[i][0].cwiseMin(edge[i][1]);
		Vector3d bb_max = edge[i][0].cwiseMax(edge[i][1]);

		hash_table.Insert(bb_min, bb_max, i, 1);
	}

	for (int i = 0; i < N; i++) {
		Vector3d bb_min = edge[i][0].cwiseMin(edge[i][1]);
		Vector3d bb_max = edge[i][0].cwiseMax(edge[i][1]);

		auto candidates = hash_table.Find(bb_min, bb_max, 1);
		for (int j = 0; j < N; j++) {
			if (BBIntersect(edge[i][0], edge[i][1], edge[j][0], edge[j][1])) {
				EXPECT_NE(std::find(candidates.begin(), candidates.end(), j), candidates.end());
				spdlog::info("Not fuck");
			} else {
				spdlog::warn("Fuck");
				// EXPECT_EQ(std::find(candidates.begin(), candidates.end(), j), candidates.end());
			}
		}
	}
}