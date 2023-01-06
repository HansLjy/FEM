#include "Collision/CCD/CubicSolver/BisectionCubicSolver.h"
#include "gtest/gtest.h"


struct FunctionCoef {
    double A, B, C, D, root;
};

const double tolerance = 1e-10;

const FunctionCoef 
    monotonic {3, -3, 1, 2, -0.55613},
    normal {2, -12, 22, -12},
    small {1, -2, -2, -3},
    large {1, 0, -0.5, +0.5};

TEST(CubicSolverTest, BisectionSolverTest) {
    BisectionCubicSolver solver(tolerance);
    
    // Monotonic increasing case
    EXPECT_NEAR(monotonic.root, solver.Solve(monotonic.A, monotonic.B, monotonic.C, monotonic.D, -1, 1), 1e-5);
    EXPECT_LT(solver.Solve(monotonic.A, monotonic.B, monotonic.C, monotonic.D, 0, 1), 0);


    // 
    EXPECT_NEAR(1.0, solver.Solve(normal.A, normal.B, normal.C, normal.D, 0.5, 1.2), 1e-10);
    EXPECT_NEAR(1.0, solver.Solve(normal.A, normal.B, normal.C, normal.D, 0.5, 2.1), 1e-10);
    EXPECT_NEAR(1.0, solver.Solve(normal.A, normal.B, normal.C, normal.D, 0.5, 2.7), 1e-10);
    EXPECT_NEAR(1.0, solver.Solve(normal.A, normal.B, normal.C, normal.D, 0.5, 3), 1e-10);

    EXPECT_NEAR(2.0, solver.Solve(normal.A, normal.B, normal.C, normal.D, 1.5, 2.5), 1e-10);
    EXPECT_NEAR(2.0, solver.Solve(normal.A, normal.B, normal.C, normal.D, 1.5, 2.7), 1e-10);
    EXPECT_NEAR(2.0, solver.Solve(normal.A, normal.B, normal.C, normal.D, 1.5, 3), 1e-10);

    EXPECT_NEAR(3.0, solver.Solve(normal.A, normal.B, normal.C, normal.D, 2.7, 3.3), 1e-10);

    EXPECT_NEAR(3.0, solver.Solve(small.A, small.B, small.C, small.D, -1, 4), 1e-10);

    EXPECT_NEAR(3.0, solver.Solve(- small.A, - small.B, - small.C, - small.D, -1, 4), 1e-10);
    EXPECT_NEAR(-1.0, solver.Solve(large.A, large.B, large.C, large.D, -2, 3), tolerance);
}