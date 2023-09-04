#pragma once

#include "DecomposedObject.hpp"
#include "TreeTrunk.hpp"
#include "unsupported/Eigen/KroneckerProduct"
#include "Render/RenderShape.hpp"

class AffineDecomposedTreeTrunk : public AffineDecomposedObject, public ProxyRenderShape, public NullCollisionShape {
public:
	explicit AffineDecomposedTreeTrunk(const json& config);
	void Initialize() override {
		AffineDecomposedObject::Initialize();
		NullCollisionShape::Precompute(this);
	}
	
	void CalculateRigidRotationInfos(const CalculateLevel& level, const Ref<const VectorXd>& x, std::vector<Matrix3d>& rotations, std::vector<MatrixXd>& rotation_gradient, std::vector<MatrixXd>& rotation_hessian) const override;

	template<AffineDecomposedObject::CalculateLevel level>
	void CalculateRigidRotationInfosTemplate(const Ref<const VectorXd>& x, std::vector<Matrix3d>& rotations, std::vector<MatrixXd>& rotation_gradient, std::vector<MatrixXd>& rotation_hessian) const;

	PROXY_RENDER_SHAPE(ProxyRenderShape)
	PROXY_COLLISION_SHAPE(NullCollisionShape)

protected:

	ReducedTreeTrunk* _tree_trunk;
	std::vector<double> _children_position;
};

void PTCGradient(const double in1[3], const double in2[3], double gradient[6]);
void PTCHessian(const double in1[3], const double in2[3], double hessian[36]);
void PTSGradient(const double in1[3], const double in2[3], double gradient[18]);
void PTSHessian(const double in1[3], const double in2[3], double hessian[108]);

template<AffineDecomposedObject::CalculateLevel level>
void AffineDecomposedTreeTrunk::CalculateRigidRotationInfosTemplate(const Ref<const VectorXd>& x, std::vector<Matrix3d>& rotations, std::vector<MatrixXd>& rotation_gradient, std::vector<MatrixXd>& rotation_hessian) const {
	const VectorXd x_tree_trunk = _tree_trunk->_base * x + _tree_trunk->_shift;
	const int num_points = x_tree_trunk.size() / 3;
	
	Vector3d e_prev = x_tree_trunk.segment<3>(0) - _tree_trunk->_x_root;
	Vector3d e_next = x_tree_trunk.segment<3>(3) - x_tree_trunk.segment<3>(0);
	Vector3d t_prev = e_prev.normalized();
	Vector3d t_next = e_next.normalized();

	Matrix3d cur_rotation = Matrix3d::Identity();
	MatrixXd cur_rotation_gradient = MatrixXd::Zero(9, 9);
	MatrixXd cur_rotation_hessian = MatrixXd::Zero(9, 81);

	Matrix<double, 9, 6> pepq;
	switch (level) {
		case AffineDecomposedObject::CalculateLevel::kGradient:
		case AffineDecomposedObject::CalculateLevel::kHessian:
			pepq.leftCols(3) = _tree_trunk->_base.middleRows(0, 3).transpose();
			pepq.rightCols(3) = (_tree_trunk->_base.middleRows(3, 3) - _tree_trunk->_base.middleRows(0, 3)).transpose();
	}

	int num_children_processed = 0;
	const double t_delta = 1.0 / (num_points - 1);
	double t_current = t_delta; // end of current segment
	for (int i = 0, i3 = 0; i < num_points - 1; i++, i3 += 3, t_current += t_delta) {
		const double C = t_prev.dot(t_next);
		const Vector3d S = t_prev.cross(t_next);
		const Matrix3d R = C * Matrix3d::Identity() + 1 / (1 + C) * S * S.transpose() + HatMatrix(S);

		Vector6d pCpe;
		Matrix<double, 6, 3> pSpe;
		if (level == AffineDecomposedObject::CalculateLevel::kGradient || level == AffineDecomposedObject::CalculateLevel::kHessian) {
			PTCGradient(e_prev.data(), e_next.data(), pCpe.data());
			PTSGradient(e_prev.data(), e_next.data(), pSpe.data());
		}
		Vector9d pCpq = pepq * pCpe;
		Matrix<double, 9, 3> pSpq = pepq * pSpe;

		Matrix6d p2Cpe2;
		Matrix<double, 6, 18> p2Spe2;
		
		if (level == AffineDecomposedObject::CalculateLevel::kGradient || level == AffineDecomposedObject::CalculateLevel::kHessian) {
			PTCHessian(e_prev.data(), e_next.data(), p2Cpe2.data());
			PTSHessian(e_prev.data(), e_next.data(), p2Spe2.data());
		}
		Matrix9d p2Cpq2 = pepq * p2Cpe2 * pepq.transpose();
		Matrix<double, 9, 27> p2Spq2 = pepq * p2Spe2 * Eigen::kroneckerProduct(Matrix3d::Identity(), pepq.transpose());
		
		Vector9d pRpq[3][3];
		Matrix9d p2Rpq2[3][3];

		const double denominator = 1 + C;
		if (level != AffineDecomposedObject::CalculateLevel::kValue) {
			for (int j = 0, j9 = 0; j < 3; j++, j9 += 9) {
				const double Sj = S(j);
				const auto& pSjpq = pSpq.col(j);
				const auto& p2Sjpq2 = p2Spq2.middleCols(j9, 9);
				pRpq[j][j] = pCpq + 2 * Sj / denominator * pSjpq - Sj * Sj / (denominator * denominator) * pCpq;
				if (level == AffineDecomposedObject::CalculateLevel::kHessian) {
					p2Rpq2[j][j] = 2 * Sj / denominator * p2Sjpq2 + (1 - Sj * Sj / (denominator * denominator)) * p2Cpq2
								+ 2 / denominator * pSjpq * pSjpq.transpose()
								- 2 * Sj / (denominator * denominator) * (pSjpq * pCpq.transpose() + pCpq * pSjpq.transpose())
								+ 2 * Sj * Sj / (denominator * denominator * denominator) * pCpq * pCpq.transpose();
				}
				for (int k = 0, k9 = 0; k < 3; k++, k9 += 9) {
					if (k == j) {
						continue;
					}
					const double Sk = S(k);
					const auto& pSkpq = pSpq.col(k);
					const auto& p2Skpq2 = p2Spq2.middleCols(k9, 9);
					
					pRpq[j][k] = Sj / denominator * pSkpq + Sk / denominator * pSjpq - Sj * Sk / (denominator * denominator) * pCpq;
					if (level == AffineDecomposedObject::CalculateLevel::kHessian) {
						p2Rpq2[j][k] = 1 / denominator * (pSjpq * pSkpq.transpose() + pSkpq * pSjpq.transpose())
									- Sj / (denominator * denominator) * (pSkpq * pCpq.transpose() + pCpq * pSkpq.transpose())
									- Sk / (denominator * denominator) * (pSjpq * pCpq.transpose() + pCpq * pSjpq.transpose())
									+ 2 * Sj * Sk / (denominator * denominator * denominator) * pCpq * pCpq.transpose()
									+ Sj / denominator * p2Skpq2 + Sk / denominator * p2Sjpq2
									- Sj * Sk / (denominator * denominator) * p2Cpq2;
					}
				}
			}
		}

		if (level != AffineDecomposedObject::CalculateLevel::kValue) {
			pRpq[0][1] -= pSpq.col(2);
			pRpq[1][0] += pSpq.col(2);
			pRpq[0][2] += pSpq.col(1);
			pRpq[2][0] -= pSpq.col(1);
			pRpq[1][2] -= pSpq.col(0);
			pRpq[2][1] += pSpq.col(0);

			if (level == AffineDecomposedObject::CalculateLevel::kHessian) {
				p2Rpq2[0][1] -= p2Spq2.middleCols(18, 9);
				p2Rpq2[1][0] += p2Spq2.middleCols(18, 9);
				p2Rpq2[0][2] += p2Spq2.middleCols(9, 9);
				p2Rpq2[2][0] -= p2Spq2.middleCols(9, 9);
				p2Rpq2[1][2] -= p2Spq2.middleCols(0, 9);
				p2Rpq2[2][1] += p2Spq2.middleCols(0, 9);
			}

		}

		MatrixXd next_rotation_gradient;
		MatrixXd next_rotation_hessian;
		if (level != AffineDecomposedObject::CalculateLevel::kValue) {
			next_rotation_gradient.resizeLike(cur_rotation_gradient);
			if (level == AffineDecomposedObject::CalculateLevel::kHessian) {
				next_rotation_hessian.resizeLike(cur_rotation_hessian);
			}
		}

		const auto& getIndex = [](int i, int j) -> int {
			return 3 * j + i;
		};

		if (level != AffineDecomposedObject::CalculateLevel::kValue) {
			for (int j = 0; j < 3; j++) {
				for (int k = 0; k < 3; k++) {
					Ref<VectorXd> pjkpq = next_rotation_gradient.col(getIndex(j, k));
					pjkpq.setZero();
					for (int l = 0; l < 3; l++) {
						pjkpq += pRpq[j][l] * cur_rotation(l, k) + R(j, l) * cur_rotation_gradient.col(getIndex(l, k));
					}
				}
			}
		}

		if (level == AffineDecomposedObject::CalculateLevel::kHessian) {
			for (int j = 0; j < 3; j++) {
				for (int k = 0; k < 3; k++) {
					Ref<MatrixXd> p2jkpq2 = next_rotation_hessian.middleCols(9 * getIndex(j, k), 9);
					p2jkpq2.setZero();
					for (int l = 0; l < 3; l++) {
						p2jkpq2 += p2Rpq2[j][l] * cur_rotation(l, k)
								+ pRpq[j][l] * cur_rotation_gradient.col(getIndex(l, k)).transpose()
								+ cur_rotation_gradient.col(getIndex(l, k)) * pRpq[j][l].transpose()
								+ R(j, l) * cur_rotation_hessian.middleCols(9 * getIndex(l, k), 9);
					}
				}
			}
		}

		cur_rotation = R * cur_rotation;
		if (level != AffineDecomposedObject::CalculateLevel::kValue) {
			cur_rotation_gradient = next_rotation_gradient;
			if (level == AffineDecomposedObject::CalculateLevel::kHessian) {
				cur_rotation_hessian = next_rotation_hessian;
			}
		}
		
		while(num_children_processed < _num_children && _children_position[num_children_processed] <= t_current) {
			const auto& rest_A = _children_rest_A[num_children_processed];
			switch (level) {
				case AffineDecomposedObject::CalculateLevel::kHessian: {
					MatrixXd child_rotation_hessian;
					child_rotation_hessian.resizeLike(cur_rotation_hessian);
					for (int j = 0; j < 3; j++) {
						for (int k = 0; k < 3; k++) {
							Ref<MatrixXd> p2jkpq2 = child_rotation_hessian.middleCols(getIndex(j, k) * 9, 9);
							p2jkpq2.setZero();
							for (int l = 0; l < 3; l++) {
								p2jkpq2 += cur_rotation_hessian.middleCols(getIndex(j, l) * 9, 9) * rest_A(l, k);
							}
						}
					}
					rotation_hessian.push_back(child_rotation_hessian);
				}
				case AffineDecomposedObject::CalculateLevel::kGradient: {
					MatrixXd child_rotation_gradient;
					child_rotation_gradient.resizeLike(cur_rotation_gradient);
					for (int j = 0; j < 3; j++) {
						for (int k = 0; k < 3; k++) {
							Ref<MatrixXd> pjkpq = child_rotation_gradient.col(getIndex(j, k));
							pjkpq.setZero();
							for (int l = 0; l < 3; l++) {
								pjkpq += cur_rotation_gradient.col(getIndex(j, l)) * rest_A(l, k);
							}
						}
					}
					rotation_gradient.push_back(child_rotation_gradient);
				}
				case AffineDecomposedObject::CalculateLevel::kValue: {
					rotations.push_back(cur_rotation * rest_A);
				}
			}
			num_children_processed++;
		}

		if (i < num_points - 2) {
			e_prev = e_next;
			t_prev = t_next;
			e_next = x_tree_trunk.segment<3>(i3 + 6) - x_tree_trunk.segment<3>(i3 + 3);
			t_next = e_next.normalized();

			if (level != AffineDecomposedObject::CalculateLevel::kValue) {
				pepq.leftCols(3) = pepq.rightCols(3);
				pepq.rightCols(3) = (_tree_trunk->_base.middleRows(i3 + 6, 3) - _tree_trunk->_base.middleRows(i3 + 3, 3)).transpose();
			}
		}
	}
}