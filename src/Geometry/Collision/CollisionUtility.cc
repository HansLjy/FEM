#include "CollisionUtility.h"
#include <functional>

#define block3(i, j) block<3, 3>(3 * i, 3 * j)
#define slice3(i, j) block<3, 1>(3 * i, j)

void CollisionUtils::PerfectElasticCollision(double v1, double v2, double m1, double m2, double& v1_after, double& v2_after) {
    if (m1 < 0) {
        v1_after = v1;
        v2_after = -v2 + 2 * v1;
    } else if (m2 < 0) {
        v1_after = -v1 + 2 * v2;
        v2_after = v2;
    } else {
        v1_after = ((m1 - m2) * v1 + 2 * m2 * v2) / (m1 + m2);
        v2_after = ((m2 - m1) * v2 + 2 * m1 * v1) / (m1 + m2);
    }
}

int GetVFDistanceType(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) {
	Vector3d e1 = face2 - face1, e2 = face3 - face1, e3 = vertex - face1;
    double A = e1.dot(e1);
    double B = e1.dot(e2);
    double C = e2.dot(e2);
    double D = e3.dot(e1);
    double E = e3.dot(e2);
    double delta = A * C - B * B;

	const double lambda3 = (C * D - B * E) / delta;
	const double lambda2 = (-B * D + A * E) / delta;
	const double lambda1 = 1 - lambda2 - lambda3;

	const double eta2 = E / C;
	const double eta3 = D / A;
	Vector3d e_tmp = face3 - face2;
	const double eta1 = e_tmp.dot(vertex - face2) / e_tmp.dot(e_tmp);

	if (lambda1 >= 0 && lambda2 >= 0 && lambda3 >= 0) {
		return 0b111;
	} else {
		if (eta2 <= 0 && eta3 <= 0) {
			return 0b001;
		}
		if (eta2 >= 1 && eta1 >= 1) {
			return 0b100;
		}
		if (eta3 >= 1 && eta1 <= 0) {
			return 0b010;
		}
		if (0 < eta2 && eta2 < 1 && lambda3 < 0) {
			return 0b101;
		}
		if (0 < eta3 && eta3 < 1 && lambda2 < 0) {
			return 0b011;
		}
		return 0b110;
	}
}

double Clamp(double x) {
	return x > 1 ? 1 : (x < 0 ? 0 : x);
}

int GetEEDistanceType(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) {
    Vector3d e1 = edge12 - edge11;
    Vector3d e2 = edge22 - edge21;
    Vector3d e3 = edge21 - edge11;
    double A = e1.dot(e1);
    double B = e2.dot(e1);
    double C = e2.dot(e2);
    double D = e3.dot(e1);
    double E = e3.dot(e2);
    double delta = A * C - B * B;

	double lambda1, lambda2;

	if (delta > 0) {
		const double CD_BE = C * D - B * E;
		const double BD_AE = B * D - A * E;

		if (CD_BE > delta) {
			if (BD_AE > delta) {
				// region 2
				lambda1 = A - B - D <= 0 ? 1 : Clamp((B + D) / A);
				lambda2 = -B + C + E <= 0 ? 1 : Clamp((B - E) / C);
			} else if (BD_AE >= 0) {
				// region 1
				lambda1 = 1;
				lambda2 = Clamp((B - E) / C);
			} else {
				// region 8
				lambda1 = (A - D) <= 0 ? 1 : Clamp(D / A);
				lambda2 = (E - B) >= 0 ? 0 : Clamp((B - E) / C);
			}
		} else if (CD_BE >= 0) {
			if (BD_AE > delta) {
				// region 3
				lambda1 = Clamp((B + D) / A);
				lambda2 = 1;
			} else if (BD_AE >= 0) {
				// region 0
				lambda1 = CD_BE / delta;
				lambda2 = BD_AE / delta;
			} else {
				// region 7
				lambda1 = Clamp(D / A);
				lambda2 = 0;
			}
		} else {
			if (BD_AE > delta) {
				// region 4
				lambda1 = (- B - D) >= 0 ? 0 : Clamp((B + D) / A);
				lambda2 = (C + E) <= 0 ? 1 : Clamp(-E / C);
			} else if (BD_AE >= 0) {
				// region 5
				lambda1 = 0;
				lambda2 = Clamp(-E / C);
			} else {
				// region 6
				lambda1 = (-D) >= 0 ? 0 : Clamp(D / A);
				lambda2 = (E) >= 0 ? 0 : Clamp(-E / C);
			}
		}
	} else {
		if (E > 0) {
			lambda1 = B > 0 ? std::min(double(1), E / B) : 0;
			lambda2 = 0;
		} else if (E < -C) {
			lambda1 = B > 0 ? 0 : std::min(double(1), (C + E) / B);
			lambda2 = 1;
		} else {
			lambda1 = 0;
			lambda2 = -E / C;
		}
	}
	return ((lambda2 < 1) << 3) + ((0 < lambda2) << 2) + ((lambda1 < 1) << 1) + (0 < lambda1);
}

double GetVLDistance(const Vector3d& vertex, const Vector3d& edge1, const Vector3d& edge2) {
	return (vertex - edge1).cross(edge2 - edge1).norm() / (edge2 - edge1).norm();
}

Vector9d GetVLDistanceGradient(const Vector3d& vertex, const Vector3d& edge1, const Vector3d& edge2) {
    const Vector3d e1 = vertex - edge1;
    const Vector3d e2 = edge2 - edge1;
    const double d = e1.cross(e2).norm() / e2.norm();

    const double A = e1.dot(e1);
    const double B = e1.dot(e2);
    const double C = e2.dot(e2);

    const double pspA = 1;
    const double pspB = - 2 * B / C;
    const double pspC = B * B / (C * C);
    
    return 1.0 / (2 * d) * (Vector9d() <<
        pspA * 2 * e1 + pspB * e2,
        - pspA * 2 * e1 + pspB * (- e1 - e2) - pspC * 2 * e2,
        pspB * e1 + pspC * 2 * e2
    ).finished();
}

Matrix9d GetVLDistanceHessian(const Vector3d& vertex, const Vector3d& edge1, const Vector3d& edge2) {
    const Vector3d e1 = vertex - edge1;
    const Vector3d e2 = edge2 - edge1;
    const double d = e1.cross(e2).norm() / e2.norm();

    const double A = e1.dot(e1);
    const double B = e1.dot(e2);
    const double C = e2.dot(e2);

    const double pspA = 1;
    const double pspB = - 2 * B / C;
    const double pspC = B * B / (C * C);

    Vector9d pspx = (Vector9d() <<
        pspA * 2 * e1 + pspB * e2,
        - pspA * 2 * e1 + pspB * (- e1 - e2) - pspC * 2 * e2,
        pspB * e1 + pspC * 2 * e2
    ).finished();
    
    Matrix2d p2spy2 = (Matrix2d() << 
        - 2 / C, 2 * B / (C * C),
        2 * B / (C * C), -2 * B * B / (C * C * C)
    ).finished();

    Matrix<double, 9, 2> pypx;
    pypx.slice3(0, 0) = e2;
    pypx.slice3(1, 0) = - e1 - e2;
    pypx.slice3(2, 0) = e1;
    pypx.slice3(0, 1) = Vector3d::Zero();
    pypx.slice3(1, 1) = -2 * e2;
    pypx.slice3(2, 1) = 2 * e2;

    Matrix9d p2spx2 = pypx * p2spy2 * pypx.transpose();
    p2spx2.block3(0, 0) += (2 * pspA) * Matrix3d::Identity();
    p2spx2.block3(0, 1) += (-2 * pspA - pspB) * Matrix3d::Identity();
    p2spx2.block3(1, 0) += (-2 * pspA - pspB) * Matrix3d::Identity();
    p2spx2.block3(0, 2) += pspB * Matrix3d::Identity();
    p2spx2.block3(2, 0) += pspB * Matrix3d::Identity();
    p2spx2.block3(1, 1) += 2 * (pspA + pspB + pspC) * Matrix3d::Identity();
    p2spx2.block3(1, 2) += (-pspB - 2 * pspC) * Matrix3d::Identity();
    p2spx2.block3(2, 1) += (-pspB - 2 * pspC) * Matrix3d::Identity();
    p2spx2.block3(2, 2) += 2 * pspC * Matrix3d::Identity();

    return 1.0 / (2 * d) * p2spx2 - 1.0 / (4 * d * d * d) * pspx * pspx.transpose();
}

double 	GetVVDistance(const Vector3d& v1, const Vector3d& v2) {
    return (v1 - v2).norm();
}

Vector6d GetVVDistanceGradient(const Vector3d& v1, const Vector3d& v2) {
	double d = (v1 - v2).norm();
    return 1.0 / (2 * d) * (Vector6d() << 2 * v1 - 2 * v2, -2 * v1 + 2 * v2).finished();
}

Matrix6d GetVVDistanceHessian(const Vector3d &v1, const Vector3d &v2) {
	static const Matrix6d p2spx2 = (Matrix6d() <<
        2, 0, 0, -2, 0, 0,
        0, 2, 0, 0, -2, 0,
        0, 0, 2, 0, 0, -2,
        -2, 0, 0, 2, 0, 0,
        0, -2, 0, 0, 2, 0,
        0, 0, -2, 0, 0, 2
    ).finished();

    double d = (v1 - v2).norm();
    Vector6d pspx = (Vector6d() << 2 * v1 - 2 * v2, -2 * v1 + 2 * v2).finished();
    
    return 1.0 / (2 * d) * p2spx2 - 1 / (4 * d * d * d) * pspx * pspx.transpose();
}

double GetVPDistance(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) {
	Vector3d e1 = face2 - face1, e2 = face3 - face1, e3 = vertex - face1;
    double A = e1.dot(e1);
    double B = e1.dot(e2);
    double C = e2.dot(e2);
    double D = e3.dot(e1);
    double E = e3.dot(e2);
    double delta = A * C - B * B;

	const double lambda3 = (C * D - B * E) / delta;
	const double lambda2 = (-B * D + A * E) / delta;

    return (vertex - (face1 + lambda3 * (face2 - face1) + lambda2 * (face3 - face1))).norm();
}


// Generated by matlab
void MediumDistanceHessian(double A, double B, double C, double D, double E, double,
                     double hessian[36]) {
    double t11;
    double t2;
    double t21;
    double t22;
    double t24;
    double t25;
    double t26;
    double t28;
    double t3;
    double t30;
    double t31;
    double t33;
    double t35;
    double t36;
    double t37;
    double t38;
    double t39;
    double t4;
    double t40;
    double t5;
    double t6;
    double t7;
    double t8;
    // DistanceHessian
    //     HESSIAN = DistanceHessian(A,B,C,D,E,F)
    //     This function was generated by the Symbolic Math Toolbox version 9.2.
    //     04-Jan-2023 16:41:47
    t2 = B * B;
    t3 = D * D;
    t4 = E * E;
    t5 = A * C;
    t6 = A * E;
    t7 = B * D;
    t8 = B * E;
    t11 = E * t5;
    t21 = t6 - t7;
    t22 = t8 - C * D;
    t24 = 1.0 / (t2 - t5);
    t6 = (D * t5 + D * t2) - B * t6 * 2.0;
    t30 = (t11 + E * t2) - C * t7 * 2.0;
    t25 = t24 * t24;
    t26 = t24 * t24 * t24;
    t28 = -(B * t24 * 2.0);
    t31 = A * t21 * t25 * 2.0;
    t33 = B * t22 * t25 * 2.0;
    t37 = t25 * t6 * 2.0;
    t38 = t25 * t30 * 2.0;
    t36 = B * t21;
    t39 = t36 * t22 * t26 * 2.0;
    t40 = t21 * t26 * t6 * 2.0;
    t35 = -(t36 * t25 * 2.0);
    t36 = -(C * t22 * t25 * 2.0);
    t6 = -(t22 * t26 * t30 * 2.0);
    hessian[0] = C * (t22 * t22) * t26 * 2.0;
    hessian[1] = t6;
    hessian[2] = t39;
    hessian[3] = t36;
    hessian[4] = t33;
    hessian[5] = 0.0;
    hessian[6] = t6;
    hessian[7] = t26 *
                (((((t7 * t11 * -6.0 + A * t2 * t4 * 3.0) + A * t4 * t5) -
                    B * t7 * t8 * 2.0) +
                    C * t2 * t3 * 3.0) +
                C * t3 * t5) *
                2.0;
    hessian[8] = t40;
    hessian[9] = t38;
    hessian[10] = t37;
    hessian[11] = 0.0;
    hessian[12] = t39;
    hessian[13] = t40;
    hessian[14] = A * (t21 * t21) * t26 * 2.0;
    hessian[15] = t35;
    hessian[16] = t31;
    hessian[17] = 0.0;
    hessian[18] = t36;
    hessian[19] = t38;
    hessian[20] = t35;
    hessian[21] = C * t24 * 2.0;
    hessian[22] = t28;
    hessian[23] = 0.0;
    hessian[24] = t33;
    hessian[25] = t37;
    hessian[26] = t31;
    hessian[27] = t28;
    hessian[28] = A * t24 * 2.0;
    hessian[29] = 0.0;
    hessian[30] = 0.0;
    hessian[31] = 0.0;
    hessian[32] = 0.0;
    hessian[33] = 0.0;
    hessian[34] = 0.0;
    hessian[35] = 0.0;
}

// Generated by matlab
void MediumDistanceGradient(double A, double B, double C, double D, double E, double,
                            double gradient[6]) {
    double t11;
    double t12;
    double t14;
    double t15;
    // DistanceGradient
    //     GRADIENT = DistanceGradient(A,B,C,D,E,F)
    //     This function was generated by the Symbolic Math Toolbox version 9.2.
    //     04-Jan-2023 16:41:47
    t11 = A * E - B * D;
    t12 = B * E - C * D;
    t14 = 1.0 / (B * B - A * C);
    t15 = t14 * t14;
    gradient[0] = t12 * t12 * t15;
    gradient[1] = t11 * t12 * t15 * -2.0;
    gradient[2] = t11 * t11 * t15;
    gradient[3] = t12 * t14 * -2.0;
    gradient[4] = t11 * t14 * 2.0;
    gradient[5] = 1.0;
}

#define pspA pspy[0]
#define pspB pspy[1]
#define pspC pspy[2]
#define pspD pspy[3]
#define pspE pspy[4]
#define pspF pspy[5]

Vector12d GetVPDistanceGradient(const Vector3d &vertex, const Vector3d &face1, const Vector3d &face2, const Vector3d &face3) {
    const Vector3d e1 = face2 - face1;
    const Vector3d e2 = face3 - face1;
    const Vector3d e3 = vertex - face1;

    const double A = e1.dot(e1),
                 B = e1.dot(e2),
                 C = e2.dot(e2),
                 D = e3.dot(e1),
                 E = e3.dot(e2),
                 F = e3.dot(e3);

	const double delta = A * C - B * B;

	const double lambda3 = (C * D - B * E) / delta;
	const double lambda2 = (-B * D + A * E) / delta;


    const double s = F + lambda3 * lambda3 * A + lambda2 * lambda2 * C - 2 * lambda3 * D - 2 * lambda2 * E + 2 * lambda2 * lambda3 * B;

    double pspy[6];
    MediumDistanceGradient(A, B, C, D, E, F, pspy);
    
    return 1.0 / (2 * sqrt(s)) * (Vector12d() <<
        pspy[3] * e1 + pspy[4] * e2 + 2 * pspy[5] * e3,
        -2 * pspy[0] * e1 - pspy[1] * (e1 + e2) - 2 * pspy[2] * e2 - pspy[3] * (e1 + e3) - pspy[4] * (e2 + e3) - 2 * pspy[5] * e3,
        2 * pspy[0] * e1 + pspy[1] * e2 + pspy[3] * e3,
        pspy[1] * e1 + 2 * pspy[2] * e2 + pspy[4] * e3
    ).finished();
}

Matrix12d GetVPDistanceHessian(const Vector3d &vertex, const Vector3d &face1, const Vector3d &face2, const Vector3d &face3) {
    const Vector3d e1 = face2 - face1;
    const Vector3d e2 = face3 - face1;
    const Vector3d e3 = vertex - face1;

    const double A = e1.dot(e1),
                 B = e1.dot(e2),
                 C = e2.dot(e2),
                 D = e3.dot(e1),
                 E = e3.dot(e2),
                 F = e3.dot(e3);

	const double delta = A * C - B * B;

	const double lambda3 = (C * D - B * E) / delta;
	const double lambda2 = (-B * D + A * E) / delta;

    const double s = F + lambda3 * lambda3 * A + lambda2 * lambda2 * C - 2 * lambda3 * D - 2 * lambda2 * E + 2 * lambda2 * lambda3 * B;
    const double d = sqrt(s);

    double pspy[6];
    MediumDistanceGradient(A, B, C, D, E, F, pspy);
    
    Matrix6d p2spy2;
    MediumDistanceHessian(A, B, C, D, E, F, p2spy2.data());

    Matrix<double, 12, 6> pypx = Matrix<double, 12, 6>::Zero();
    pypx.slice3(1, 0) = -2 * e1;
    pypx.slice3(2, 0) = 2 * e1;
    pypx.slice3(1, 1) = - e1 - e2;
    pypx.slice3(2, 1) = e2;
    pypx.slice3(3, 1) = e1;
    pypx.slice3(1, 2) = -2 * e2;
    pypx.slice3(3, 2) = 2 * e2;
    pypx.slice3(0, 3) = e1;
    pypx.slice3(1, 3) = - e3 - e1;
    pypx.slice3(2, 3) = e3;
    pypx.slice3(0, 4) = e2;
    pypx.slice3(1, 4) = - e2 - e3;
    pypx.slice3(3, 4) = e3;
    pypx.slice3(0, 5) = 2 * e3;
    pypx.slice3(1, 5) = -2 * e3;

    Matrix12d p2spx2 = pypx * p2spy2 * pypx.transpose();
    p2spx2.block3(0, 0) += 2 * pspF * Matrix3d::Identity();
    p2spx2.block3(0, 1) += (- pspD - pspE - 2 * pspF) * Matrix3d::Identity();
    p2spx2.block3(1, 0) += (- pspD - pspE - 2 * pspF) * Matrix3d::Identity();
    p2spx2.block3(0, 2) += pspD * Matrix3d::Identity();
    p2spx2.block3(2, 0) += pspD * Matrix3d::Identity();
    p2spx2.block3(0, 3) += pspE * Matrix3d::Identity();
    p2spx2.block3(3, 0) += pspE * Matrix3d::Identity();
    p2spx2.block3(1, 1) += (2 * pspA + 2 * pspB + 2 * pspC + 2 * pspD + 2 * pspE + 2 * pspF) * Matrix3d::Identity();
    p2spx2.block3(1, 2) += (-2 * pspA - pspB - pspD) * Matrix3d::Identity();
    p2spx2.block3(2, 1) += (-2 * pspA - pspB - pspD) * Matrix3d::Identity();
    p2spx2.block3(1, 3) += (- pspB - 2 * pspC - pspE) * Matrix3d::Identity();
    p2spx2.block3(3, 1) += (- pspB - 2 * pspC - pspE) * Matrix3d::Identity();
    p2spx2.block3(2, 2) += 2 * pspA * Matrix3d::Identity();
    p2spx2.block3(2, 3) += pspB * Matrix3d::Identity();
    p2spx2.block3(3, 2) += pspB * Matrix3d::Identity();
    p2spx2.block3(3, 3) += 2 * pspC * Matrix3d::Identity();

    Vector12d pspx = (Vector12d() <<
        pspy[3] * e1 + pspy[4] * e2 + 2 * pspy[5] * e3,
        -2 * pspy[0] * e1 - pspy[1] * (e1 + e2) - 2 * pspy[2] * e2 - pspy[3] * (e1 + e3) - pspy[4] * (e2 + e3) - 2 * pspy[5] * e3,
        2 * pspy[0] * e1 + pspy[1] * e2 + pspy[3] * e3,
        pspy[1] * e1 + 2 * pspy[2] * e2 + pspy[4] * e3
    ).finished();


    return 1 / (2 * d) * p2spx2 - 1 / (4 * s * d) * pspx * pspx.transpose();
}

double GetLLDistance(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) {
	Vector3d e1 = edge12 - edge11;
    Vector3d e2 = edge22 - edge21;
    Vector3d e3 = edge21 - edge11;
    double A = e1.dot(e1);
    double B = e2.dot(e1);
    double C = e2.dot(e2);
    double D = e3.dot(e1);
    double E = e3.dot(e2);
    double delta = A * C - B * B;

	const double lambda1 = (C * D - B * E) / delta;
	const double lambda2 = (B * D - A * E) / delta;

    return ((edge11 + lambda1 * (edge12 - edge11)) - (edge21 + lambda2 * (edge22 - edge21))).norm();
}

Vector12d GetLLDistanceGradient(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) {
    const Vector3d e1 = edge12 - edge11;
    const Vector3d e2 = edge22 - edge21;
    const Vector3d e3 = edge21 - edge11;

    const double A = e1.dot(e1),
                 B = e1.dot(e2),
                 C = e2.dot(e2),
                 D = e3.dot(e1),
                 E = e3.dot(e2),
                 F = e3.dot(e3);
    double delta = A * C - B * B;

	const double lambda1 = (C * D - B * E) / delta;
	const double lambda2 = (B * D - A * E) / delta;
    
    const double s = F + lambda1 * lambda1 * A + lambda2 * lambda2 * C - 2 * lambda1 * D + 2 * lambda2 * E - 2 * lambda1 * lambda2 * B;

    double pspy[6];
    MediumDistanceGradient(A, B, C, D, E, F, pspy);

    return 1 / (2 * sqrt(s)) * (Vector12d() <<
        -2 * pspA * e1 - pspB * e2 - pspD * (e1 + e3) - pspE * e2 - 2 * pspF * e3,
        2 * pspA * e1 + pspB * e2 + pspD * e3,
        -pspB * e1 - 2 * pspC * e2 + pspD * e1 + pspE * (e2 - e3) + 2 * pspF * e3,
        pspB * e1 + 2 * pspC * e2 + pspE * e3
    ).finished();
}

Matrix12d GetLLDistanceHessian(const Vector3d &edge11, const Vector3d &edge12, const Vector3d &edge21, const Vector3d &edge22) {
    const Vector3d e1 = edge12 - edge11;
    const Vector3d e2 = edge22 - edge21;
    const Vector3d e3 = edge21 - edge11;

    const double A = e1.dot(e1),
                 B = e1.dot(e2),
                 C = e2.dot(e2),
                 D = e3.dot(e1),
                 E = e3.dot(e2),
                 F = e3.dot(e3);

    double delta = A * C - B * B;

	const double lambda1 = (C * D - B * E) / delta;
	const double lambda2 = (B * D - A * E) / delta;
    
    const double s = F + lambda1 * lambda1 * A + lambda2 * lambda2 * C - 2 * lambda1 * D + 2 * lambda2 * E - 2 * lambda1 * lambda2 * B;
    const double d = sqrt(s);

    double pspy[6];
    MediumDistanceGradient(A, B, C, D, E, F, pspy);

    Vector12d pspx = (Vector12d() <<
        -2 * pspA * e1 - pspB * e2 - pspD * (e1 + e3) - pspE * e2 - 2 * pspF * e3,
        2 * pspA * e1 + pspB * e2 + pspD * e3,
        -pspB * e1 - 2 * pspC * e2 + pspD * e1 + pspE * (e2 - e3) + 2 * pspF * e3,
		pspB * e1 + 2 * pspC * e2 + pspE * e3
    ).finished();

    Matrix<double, 12, 6> pypx = Matrix<double, 12, 6>::Zero();
    pypx.slice3(0, 0) = - 2 * e1;
    pypx.slice3(1, 0) = 2 * e1;
    pypx.slice3(0, 1) = - e2;
    pypx.slice3(1, 1) = e2;
    pypx.slice3(2, 1) = - e1;
    pypx.slice3(3, 1) = e1;
    pypx.slice3(2, 2) = -2 * e2;
    pypx.slice3(3, 2) = 2 * e2;
    pypx.slice3(0, 3) = - e1 - e3;
    pypx.slice3(1, 3) = e3;
    pypx.slice3(2, 3) = e1;
    pypx.slice3(0, 4) = - e2;
    pypx.slice3(2, 4) = e2 - e3;
    pypx.slice3(3, 4) = e3;
    pypx.slice3(0, 5) = - 2 * e3;
    pypx.slice3(2, 5) = 2 * e3;

    Matrix6d p2spy2;
    MediumDistanceHessian(A, B, C, D, E, F, p2spy2.data());

    Matrix12d p2spx2 = pypx * p2spy2 * pypx.transpose();
    p2spx2.block3(0, 0) += (2 * pspA + 2 * pspD + 2 * pspF) * Matrix3d::Identity();
    p2spx2.block3(0, 1) += (-2 * pspA - pspD) * Matrix3d::Identity();
    p2spx2.block3(1, 0) += (-2 * pspA - pspD) * Matrix3d::Identity();
    p2spx2.block3(0, 2) += (pspB - pspD + pspE - 2 * pspF) * Matrix3d::Identity();
    p2spx2.block3(2, 0) += (pspB - pspD + pspE - 2 * pspF) * Matrix3d::Identity();
    p2spx2.block3(0, 3) += (- pspB - pspE) * Matrix3d::Identity();
    p2spx2.block3(3, 0) += (- pspB - pspE) * Matrix3d::Identity();
    p2spx2.block3(1, 1) += 2 * pspA * Matrix3d::Identity();
    p2spx2.block3(1, 2) += (- pspB + pspD) * Matrix3d::Identity();
    p2spx2.block3(2, 1) += (- pspB + pspD) * Matrix3d::Identity();
    p2spx2.block3(1, 3) += pspB * Matrix3d::Identity();
    p2spx2.block3(3, 1) += pspB * Matrix3d::Identity();
    p2spx2.block3(2, 2) += (2 * pspC - 2 * pspE + 2 * pspF) * Matrix3d::Identity();
    p2spx2.block3(2, 3) += (-2 * pspC + pspE) * Matrix3d::Identity();
    p2spx2.block3(3, 2) += (-2 * pspC + pspE) * Matrix3d::Identity();
    p2spx2.block3(3, 3) += 2 * pspC * Matrix3d::Identity();

    return 1 / (2 * d) * p2spx2 - 1 / (4 * s * d) * pspx * pspx.transpose();
}

#undef pspA
#undef pspB
#undef pspC
#undef pspD
#undef pspE
#undef pspF

#define CLASSIFY_VF(CASE111, CASE110, CASE101, CASE011, CASE001, CASE010, CASE100)\
    switch (distance_type) {\
        case 0b111:\
            CASE111\
            break;\
        case 0b110:\
            CASE110\
            break;\
        case 0b101:\
            CASE101\
            break;\
        case 0b011:\
            CASE011\
            break;\
        case 0b001:\
            CASE001\
            break;\
        case 0b010:\
            CASE010\
            break;\
        case 0b100:\
            CASE100\
            break;\
    }

double GetVFDistance(int distance_type, const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) {
	double d;
	CLASSIFY_VF(
        d = GetVPDistance(vertex, face1, face2, face3);,
        d = GetVLDistance(vertex, face2, face3);,
        d = GetVLDistance(vertex, face1, face3);,
        d = GetVLDistance(vertex, face1, face2);,
        d = GetVVDistance(vertex, face1);,
        d = GetVVDistance(vertex, face2);,
        d = GetVVDistance(vertex, face3);
    )
	return d;
}

double GetVFDistance(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) {
	auto distance_type = GetVFDistanceType(vertex, face1, face2, face3);
    return GetVFDistance(distance_type, vertex, face1, face2, face3);
}


const std::array<int, 9> index110 = {0, 1, 2, 6, 7, 8, 9, 10, 11};
const std::array<int, 9> index101 = {0, 1, 2, 3, 4, 5, 9, 10, 11};
const std::array<int, 9> index011 = {0, 1, 2, 3, 4, 5, 6, 7, 8};
const std::array<int, 6> index001 = {0, 1, 2, 3, 4, 5};
const std::array<int, 6> index010 = {0, 1, 2, 6, 7, 8};
const std::array<int, 6> index100 = {0, 1, 2, 9, 10, 11};

Vector12d GetVFDistanceGradient(int distance_type, const Vector3d &vertex, const Vector3d &face1, const Vector3d &face2, const Vector3d &face3) {
    Vector12d pdpx = Vector12d::Zero();
    CLASSIFY_VF(
        pdpx = GetVPDistanceGradient(vertex, face1, face2, face3);,
        pdpx(index110) = GetVLDistanceGradient(vertex, face2, face3);,
        pdpx(index101) = GetVLDistanceGradient(vertex, face1, face3);,
        pdpx(index011) = GetVLDistanceGradient(vertex, face1, face2);,
        pdpx(index001) = GetVVDistanceGradient(vertex, face1);,
        pdpx(index010) = GetVVDistanceGradient(vertex, face2);,
        pdpx(index100) = GetVVDistanceGradient(vertex, face3);
    )
    return pdpx;
}

Vector12d GetVFDistanceGradient(const Vector3d &vertex, const Vector3d &face1, const Vector3d &face2, const Vector3d &face3) {
	auto distance_type = GetVFDistanceType(vertex, face1, face2, face3);
	return GetVFDistanceGradient(distance_type, vertex, face1, face2, face3);
}

Matrix12d GetVFDistanceHessian(int distance_type, const Vector3d &vertex, const Vector3d &face1, const Vector3d &face2, const Vector3d &face3) {
    Matrix12d p2dpx2 = Matrix12d::Zero();
    CLASSIFY_VF(
        p2dpx2 = GetVPDistanceHessian(vertex, face1, face2, face3);,
        p2dpx2(index110, index110) = GetVLDistanceHessian(vertex, face2, face3);,
        p2dpx2(index101, index101) = GetVLDistanceHessian(vertex, face1, face3);,
        p2dpx2(index011, index011) = GetVLDistanceHessian(vertex, face1, face2);,
        p2dpx2(index001, index001) = GetVVDistanceHessian(vertex, face1);,
        p2dpx2(index010, index010) = GetVVDistanceHessian(vertex, face2);,
        p2dpx2(index100, index100) = GetVVDistanceHessian(vertex, face3);
    )
    return p2dpx2;
}

Matrix12d GetVFDistanceHessian(const Vector3d &vertex, const Vector3d &face1, const Vector3d &face2, const Vector3d &face3) {
	auto distance_type = GetVFDistanceType(vertex, face1, face2, face3);
	return GetVFDistanceHessian(distance_type, vertex, face1, face2, face3);
}

#define CLASSIFY_EE(CASE1111, CASE1110, CASE1101, CASE1011, CASE0111, CASE1010, CASE1001, CASE0110, CASE0101) \
    switch(distance_type) {\
        case 0b1111:\
            CASE1111\
            break;\
        case 0b1110:\
            CASE1110\
            break;\
        case 0b1101:\
            CASE1101\
            break;\
        case 0b1011:\
            CASE1011\
            break;\
        case 0b0111:\
            CASE0111\
            break;\
        case 0b1010:\
            CASE1010\
            break;\
        case 0b1001:\
            CASE1001\
            break;\
        case 0b0110:\
            CASE0110\
            break;\
        case 0b0101:\
            CASE0101\
            break;\
    }

double GetEEDistance(int distance_type, const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) {
    double d;
    CLASSIFY_EE(
        d = GetLLDistance(edge11, edge12, edge21, edge22);,
        d = GetVLDistance(edge11, edge21, edge22);,
        d = GetVLDistance(edge12, edge21, edge22);,
        d = GetVLDistance(edge21, edge11, edge12);,
        d = GetVLDistance(edge22, edge11, edge12);,
        d = GetVVDistance(edge11, edge21);,
        d = GetVVDistance(edge12, edge21);,
        d = GetVVDistance(edge11, edge22);,
        d = GetVVDistance(edge12, edge22);
    )
    return d;
}

double GetEEDistance(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) {
	auto distance_type = GetEEDistanceType(edge11, edge12, edge21, edge22);
	return GetEEDistance(distance_type, edge11, edge12, edge21, edge22);
}

const std::array<int, 9> index1110 = {0, 1, 2, 6, 7, 8, 9, 10, 11};
const std::array<int, 9> index1101 = {3, 4, 5, 6, 7, 8, 9, 10, 11};
const std::array<int, 9> index1011 = {6, 7, 8, 0, 1, 2, 3, 4, 5};
const std::array<int, 9> index0111 = {9, 10, 11, 0, 1, 2, 3, 4, 5};
const std::array<int, 6> index1010 = {0, 1, 2, 6, 7, 8};
const std::array<int, 6> index1001 = {3, 4, 5, 6, 7, 8};
const std::array<int, 6> index0110 = {0, 1, 2, 9, 10, 11};
const std::array<int, 6> index0101 = {3, 4, 5, 9, 10, 11};

Vector12d GetEEDistanceGradient(int distance_type, const Vector3d &edge11, const Vector3d &edge12, const Vector3d &edge21, const Vector3d &edge22) {
    Vector12d pdpx = Vector12d::Zero();
    CLASSIFY_EE(
        pdpx            = GetLLDistanceGradient(edge11, edge12, edge21, edge22);,
        pdpx(index1110) = GetVLDistanceGradient(edge11, edge21, edge22);,
        pdpx(index1101) = GetVLDistanceGradient(edge12, edge21, edge22);,
        pdpx(index1011) = GetVLDistanceGradient(edge21, edge11, edge12);,
        pdpx(index0111) = GetVLDistanceGradient(edge22, edge11, edge12);,
        pdpx(index1010) = GetVVDistanceGradient(edge11, edge21);,
        pdpx(index1001) = GetVVDistanceGradient(edge12, edge21);,
        pdpx(index0110) = GetVVDistanceGradient(edge11, edge22);,
        pdpx(index0101) = GetVVDistanceGradient(edge12, edge22);
    )
    return pdpx;
}

Vector12d GetEEDistanceGradient(const Vector3d &edge11, const Vector3d &edge12, const Vector3d &edge21, const Vector3d &edge22) {
	auto distance_type = GetEEDistanceType(edge11, edge12, edge21, edge22);
	return GetEEDistanceGradient(distance_type, edge11, edge12, edge21, edge22);
}

Matrix12d GetEEDistanceHessian(int distance_type, const Vector3d &edge11, const Vector3d &edge12, const Vector3d &edge21, const Vector3d &edge22) {
    Matrix12d p2dpx2 = Matrix12d::Zero();
    CLASSIFY_EE(
        p2dpx2                       = GetLLDistanceHessian(edge11, edge12, edge21, edge22);,
        p2dpx2(index1110, index1110) = GetVLDistanceHessian(edge11, edge21, edge22);,
        p2dpx2(index1101, index1101) = GetVLDistanceHessian(edge12, edge21, edge22);,
        p2dpx2(index1011, index1011) = GetVLDistanceHessian(edge21, edge11, edge12);,
        p2dpx2(index0111, index0111) = GetVLDistanceHessian(edge22, edge11, edge12);,
        p2dpx2(index1010, index1010) = GetVVDistanceHessian(edge11, edge21);,
        p2dpx2(index1001, index1001) = GetVVDistanceHessian(edge12, edge21);,
        p2dpx2(index0110, index0110) = GetVVDistanceHessian(edge11, edge22);,
        p2dpx2(index0101, index0101) = GetVVDistanceHessian(edge12, edge22);
    )
    return p2dpx2;
}

Matrix12d GetEEDistanceHessian(const Vector3d &edge11, const Vector3d &edge12, const Vector3d &edge21, const Vector3d &edge22) {
	auto distance_type = GetEEDistanceType(edge11, edge12, edge21, edge22);
	return GetEEDistanceHessian(distance_type, edge11, edge12, edge21, edge22);
}