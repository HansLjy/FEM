#include "RenderShape.h"

DEFINE_CLONE(RenderShape, ReducedRenderShape)

void ReducedRenderShape::GetSurface(const Object &object, MatrixXd &vertices, MatrixXi &topos) const {
	const auto& reduced_obj = dynamic_cast<const ReducedObject&>(object);
	reduced_obj._proxy->GetRenderShape(vertices, topos);
}