#include "gtest/gtest.h"
#include "FileIO.hpp"
#include "TopoUtil.hpp"
#include "PDEnergyModel/PDClothEnergyModel.hpp"

TEST(PDEnergyTest, QuadraticBendingTest) {
	VectorXd x;
	MatrixXi topo;
	FileIOUtils::ReadMesh("/plane_large.obj", x, topo);
	auto internal_edge_topo = TopoUtil::GetInternalEdge(topo);
	double stiffness = 1;

	SparseMatrixXd Q;
	PDEnergyModelFunction::InitQuadraticBendingMatrix(x, internal_edge_topo, stiffness, Q);
	Eigen::LDLT<MatrixXd> ldlt(Q.toDense());
	EXPECT_TRUE(Q.isApprox(Q.transpose()));
	EXPECT_TRUE(ldlt.info() != Eigen::NumericalIssue);
}