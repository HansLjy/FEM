//
// Created by hansljy on 11/11/22.
//

#ifndef FEM_TREETRUNKSHAPE_H
#define FEM_TREETRUNKSHAPE_H

#include "RenderShape/RenderShape.h"

class TreeTrunk;
class TreeTrunkShape : public RenderShape {
public:
    TreeTrunkShape(double radius_max, double radius_min);
	void Bind(const Object &obj) override;
    void GetSurface(Eigen::MatrixXd &vertices, Eigen::MatrixXi &topos) const override;
	
protected:
    const double _radius_max;
    const double _radius_min;
	const TreeTrunk* _tree_trunk;
};

#endif //FEM_TREETRUNKSHAPE_H
