//
// Created by hansljy on 10/11/22.
//

#include <gtest/gtest.h>
#include "Object.h"
#include "System.h"
#include "Shape.h"

class StubShape : public Shape {
    void GetSurface(const Object &object, Eigen::MatrixXd &vertices, Eigen::MatrixXi &topos) const override {}
    DERIVED_DECLARE_CLONE(Shape)
};

DEFINE_CLONE(Shape, StubShape)

class StubObject : public Object {
public:
    StubObject() : Object(VectorXd()), _uid(++count) {
        _x.resize(_uid);
        _x.setConstant(_uid);
        _v.resize(_uid);
        _v.setConstant(-_uid);
    }

    void GetMass(COO &coo, int x_offset, int y_offset) const override {}
    double GetPotential() const override {}
    VectorXd GetPotentialGradient() const override {}
    void GetPotentialHessian(COO &coo, int x_offset, int y_offset) const override {
        for (int i = 0; i < _uid; i++) {
            coo.push_back(Tripletd(i + x_offset, i + y_offset, _uid));
        }
    }

    const void GetShape(Eigen::MatrixXd &vertices, Eigen::MatrixXi &topo) const override {

    }

    DERIVED_DECLARE_CLONE(Object)

public:
    static int count;
    int _uid;
};

int StubObject::count = 0;

DEFINE_CLONE(Object, StubObject)

TEST(SystemTest, SystemIOTest) {
    System system;
    StubObject rod, ball, floor;

    int rod_idx = system.AddObject(rod, "Rod");
    int ball_idx = system.AddObject(ball, "Ball");
    int floor_idx = system.AddObject(floor, "Floor");

    EXPECT_EQ(-1, system.AddObject(rod, "Rod"));        // duplicate objects

    EXPECT_EQ(rod_idx, system.GetIndex("Rod"));
    EXPECT_EQ(ball_idx, system.GetIndex("Ball"));
    EXPECT_EQ(floor_idx, system.GetIndex("Floor"));

    SparseMatrixXd hessian;
    system.GetEnergyHessian(hessian);
    std::cout << hessian.toDense();

    system.DeleteObject(ball_idx);
    EXPECT_EQ(-1, system.GetIndex("Ball"));             // deleted objects
    EXPECT_EQ(false, system.DeleteObject("Ball"));      // deleted objects
    EXPECT_EQ(false, system.DeleteObject(ball_idx));    // deleted objects

    EXPECT_EQ(-1, system.GetIndex("Fuck"));             // non-existent objects

    for (int i = 0; i < 10; i++) {
        system.AddObject(rod, std::to_string(i));       // out of capacity;
    }
}