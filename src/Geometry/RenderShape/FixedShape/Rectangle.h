#pragma once

#include "RenderShape/RenderShape.h"

class RectangleRenderShape : public FixedRenderShape {
public:
    RectangleRenderShape(const json& config);
    RectangleRenderShape(const Vector3d& min_point, const Vector3d& max_point);

protected:
    static MatrixXd GenerateVertices(const Vector3d& min_point, const Vector3d& max_point);
    static MatrixXi GenerateTopos();
};