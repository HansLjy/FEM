//
// Created by hansljy on 10/14/22.
//

#include "gtest/gtest.h"
#include "Curve/InextensibleCurve.h"
#include "Constraint/SampledObject/FixedPointConstraint.h"

class InextensibleCurveForTest : public InextensibleCurve {
public:
    InextensibleCurveForTest(const Vector3d &start, const Vector3d &end, int num_segments, double rho, double alpha)
            : Object(Curve::GetX(start, end, num_segments)), InextensibleCurve(rho, alpha, alpha, start, end, num_segments) {}
    FRIEND_TEST(ConstraintTest, FixedPointTest);
};



TEST(ConstraintTest, FixedPointTest) {
    Vector3d start, end;
    start << 0, 0, 0;
    end << 0, 0, 1;
    const int num_segments = 1;
    InextensibleCurveForTest curve(start, end, num_segments, 1, 0.1);
    FixedPointConstraint fixed_point(start, 0, 0);
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