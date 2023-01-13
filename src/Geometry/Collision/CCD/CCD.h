//
// Created by hansljy on 11/23/22.
//

#ifndef FEM_CCD_H
#define FEM_CCD_H

#include "EigenAll.h"
#include "Pattern.h"

class CCD {
public:
    // <- return the exact collision time. Negative value indicates non-intersection
    virtual double EdgeEdgeCollision(const Vector3d& x11, const Vector3d& x12, const Vector3d& x21, const Vector3d& x22,
                                     const Vector3d& v11, const Vector3d& v12, const Vector3d& v21, const Vector3d& v22) = 0;

    // <- return the exact collision time. Negative value indicates non-intersection
    virtual double VertexFaceCollision(const Vector3d& x, const Vector3d& x1, const Vector3d& x2, const Vector3d& x3,
                                       const Vector3d& v, const Vector3d& v1, const Vector3d& v2, const Vector3d& v3) = 0;

    virtual ~CCD() = default;
};

DECLARE_XXX_FACTORY(CCD)

#endif //FEM_CCD_H
