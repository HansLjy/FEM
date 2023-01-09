#pragma once

#include "Collision/CollisionShape/CollisionShape.h"

class RectangleCollisionShape : public FixedCollisionShape {
public:
    explicit RectangleCollisionShape(const json& config);
    RectangleCollisionShape(const Vector3d& min_point, const Vector3d& max_point);

protected:
    static MatrixXd GenerateVertices(const Vector3d& min_point, const Vector3d& max_point);
    static MatrixXi GenerateEdgeTopo();
    static MatrixXi GenerateFaceTopo();
};