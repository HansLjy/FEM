#ifndef EIGEN_ALL_H
#define EIGEN_ALL_H

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <fstream>

using Eigen::Matrix;
using Eigen::Vector;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
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

using Eigen::Ref;

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
Vector3d SkewVector(const Matrix3d& mat);
Vector3d FindPerpendicular(const Vector3d& vec);
Matrix<double, 9, 3> GetVecHatMatrix();

template<int dim>
Matrix<double, dim, dim> PositiveProject(const Eigen::Matrix<double, dim, dim>& matrix) {
    typedef Eigen::Matrix<double, dim, dim> MatrixOfSize;
    typedef Eigen::Vector<double, dim> VectorOfSize;
    Eigen::SelfAdjointEigenSolver<MatrixOfSize> eigens(matrix);
    MatrixOfSize eigen_vectors = eigens.eigenvectors();
    VectorOfSize eigen_values = eigens.eigenvalues();
    for (int i = 0; i < dim; i++) {
        if (eigen_values(i) < 0) {
            eigen_values(i) = 0;
        }
    }
    return eigen_vectors * eigen_values.asDiagonal() * eigen_vectors.transpose();
}

void SparseToCOO(const SparseMatrixXd& mat, COO& coo, int offset_x, int offset_y);

inline void DenseToCOO(const Ref<const MatrixXd>& mat, COO& coo, int offset_x, int offset_y) {
	for (int i = 0; i < mat.rows(); i++) {
		for (int j = 0; j < mat.cols(); j++) {
			coo.push_back(Tripletd(offset_x + i, offset_y + j, mat(i, j)));
		}
	}
}

#include "fstream"

template<class Matrix>
void write_binary(std::ofstream &out, const Matrix &matrix) {
    typename Matrix::Index rows = matrix.rows(), cols = matrix.cols();
    out.write((char *) (&rows), sizeof(typename Matrix::Index));
    out.write((char *) (&cols), sizeof(typename Matrix::Index));
    out.write((char *) matrix.data(), rows * cols * sizeof(typename Matrix::Scalar));
}

template<class MatrixType>
void read_binary(std::ifstream &in, MatrixType &matrix) {
    typename MatrixType::Index rows = 0, cols = 0;
    in.read((char *) (&rows), sizeof(typename MatrixType::Index));
    in.read((char *) (&cols), sizeof(typename MatrixType::Index));
    matrix.resize(rows, cols);
    in.read((char *) matrix.data(), rows * cols * sizeof(typename MatrixType::Scalar));
}

template <typename Mat>
Mat VConcat(const Mat& A, const Mat& B) {
	Mat res(A.rows() + B.rows(), A.cols());
	res << A, B;
	return res;
}

template <typename ElementType, int cols>
Eigen::MatrixXd StackVector(const Eigen::Ref<const Eigen::MatrixXd>& x) {
	const int rows = x.size() / cols;
	return Eigen::Map<Eigen::Matrix<ElementType, Eigen::Dynamic, Eigen::Dynamic>, 0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>((ElementType*)(x.data()), rows, cols, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>(x.innerStride(), cols * x.innerStride()));
}


#endif