#ifndef EIGEN_ALL_H
#define EIGEN_ALL_H

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <Eigen/Core>

using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Matrix4d;
using Eigen::Vector4d;
using Eigen::MatrixX;
using Eigen::VectorX;
using Eigen::MatrixXi;

using Eigen::RowVector3i;
using Eigen::RowVector2i;
using Eigen::RowVector4i;

using Eigen::Dynamic;

typedef Eigen::Vector<double, 5> Vector5d;
typedef Eigen::Matrix<double, 5, 5> Matrix5d;
typedef Eigen::Vector<double, 6> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Vector<double, 7> Vector7d;
typedef Eigen::Matrix<double, 7, 7> Matrix7d;
typedef Eigen::Vector<double, 9> Vector9d;
typedef Eigen::Matrix<double, 9, 9> Matrix9d;
typedef Eigen::Vector<double, 12> Vector12d;
typedef Eigen::Matrix<double, 12, 12> Matrix12d;
typedef Eigen::Matrix<double, 12, 9> Matrix12x9d;
typedef Eigen::Vector<double, 1> Vector1d;
typedef Eigen::SparseMatrix<double> SparseMatrixXd;
typedef Eigen::Triplet<double> Tripletd;
typedef std::vector<Tripletd> COO;

using Eigen::AngleAxisd;

typedef Eigen::Quaternion<double> Quaterniond;

Matrix3d HatMatrix(const Vector3d& vec);
Vector3d FindPerpendicular(const Vector3d& vec);
Matrix<double, 9, 3> GetVecHatMatrix();

#endif