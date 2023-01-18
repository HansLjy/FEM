//
// Created by hansljy on 10/27/22.
//

#ifndef FEM_CLOTHSHAPE_H
#define FEM_CLOTHSHAPE_H

#include "RenderShape/RenderShape.h"

class Cloth;

class ClothShape : public RenderShape {
public:
	void Bind(const Object &obj) override;
	void GetSurface(MatrixXd &vertices, MatrixXi &topos) const override;
    ~ClothShape() override = default;

protected:
	const Cloth* _cloth;
};

#endif //FEM_CLOTHSHAPE_H
