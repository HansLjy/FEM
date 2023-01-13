#include "CollisionShape.h"
#include "FixedCollisionShape/RectangleCollisionShape.h"

BEGIN_DEFINE_XXX_FACTORY(FixedCollisionShape)
    ADD_PRODUCT("rectangle", RectangleCollisionShape)
END_DEFINE_XXX_FACTORY


void AssembleCollisionHessian(
	const Object& obj1, const Object& obj2,
	const Ref<const Matrix3d>& local_hessian,
	const CollisionAssemblerType& type1, const CollisionAssemblerType& type2,
	const int offset1, const int offset2,
	const int index1, const int index2,
	COO& coo
) {
	if (type1 == CollisionAssemblerType::kNull || type2 == CollisionAssemblerType::kNull) {
		return;
	}
	switch (type1) {
		case CollisionAssemblerType::kIndex: {
			switch (type2) {
				case CollisionAssemblerType::kIndex: {
					const int base_offset1 = offset1 + 3 * index1,
							  base_offset2 = offset2 + 3 * index2;
					for (int i = 0; i < 3; i++) {
						for (int j = 0; j < 3; j++) {
							coo.push_back(Tripletd(base_offset1 + i, base_offset2 + j, local_hessian(i, j)));
						}
					}
					break;
				}
				default: {
					throw std::logic_error("Unimplemented");
				}
			}
			break;
		}
		default: {
			throw std::logic_error("Unimplemented");
		}
	}
}