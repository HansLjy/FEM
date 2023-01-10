//
// Created by hansljy on 11/23/22.
//

#ifndef FEM_SIMPLECCD_H
#define FEM_SIMPLECCD_H

#include "CCD.h"
#include "JsonUtil.h"
#include "CubicSolver/CubicSolver.h"

class SimpleCCD : public CCD {
public:
    explicit SimpleCCD(const json& config);

    double EdgeEdgeCollision(const Eigen::Vector3d &x11, const Eigen::Vector3d &x12, const Eigen::Vector3d &x21, const Eigen::Vector3d &x22, const Eigen::Vector3d &v11, const Eigen::Vector3d &v12, const Eigen::Vector3d &v21, const Eigen::Vector3d &v22) override;
    double VertexFaceCollision(const Eigen::Vector3d &x, const Eigen::Vector3d &x1, const Eigen::Vector3d &x2, const Eigen::Vector3d &x3, const Eigen::Vector3d &v, const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, const Eigen::Vector3d &v3) override;

protected:
    bool CheckVertexInFace(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3);
    bool CheckEdgeIntersection(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22);

    double _epsilon;
    CubicSolver* _cubic_solver;
};

#endif //FEM_SIMPLECCD_H
