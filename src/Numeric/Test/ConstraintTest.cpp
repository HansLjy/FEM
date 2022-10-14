//
// Created by hansljy on 10/14/22.
//

#include "Constraint/Curve/FixedPoint.h"
#include "Constraint/Curve/InextensibleCurve.h"
#include "gtest/gtest.h"

class CurveForTest : public Curve {
public:
    CurveForTest(const Vector3d &start, const Vector3d &end, int num_segments, double total_mass, double alpha)
            : Curve(start, end, num_segments, total_mass, alpha) {}
    FRIEND_TEST(ConstraintTest, FixedPointTest);
};



TEST(ConstraintTest, FixedPointTest) {
    Vector3d start, end;
    start << 0, 0, 0;
    end << 0, 0, 1;
    const int num_segments = 1;
    CurveForTest curve(start, end, num_segments, 1, 0.1);
    FixedPoint fixed_point(curve, 0, 1, end);
    fixed_point.SetOffset(0, 0);

    VectorXd x = curve._x;

    Vector3d constraint = fixed_point.GetValue(x);
    for (int i = 0; i < 3; i++) {
        EXPECT_DOUBLE_EQ(constraint(i), 0);
    }


    Vector3d delta = Vector3d::Random();
    x.block<3, 1>(0, 0) += delta;
    constraint = fixed_point.GetValue(x);
    for (int i = 0; i < 3; i++) {
        EXPECT_DOUBLE_EQ(constraint(i), 0);
    }
    x.block<3, 1>(3, 0) += delta;
    constraint = fixed_point.GetValue(x);
    for (int i = 0; i < 3; i++) {
        EXPECT_DOUBLE_EQ(constraint(i), delta(i));
    }
}