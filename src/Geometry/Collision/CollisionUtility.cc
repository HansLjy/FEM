#include "CollisionUtility.h"

#define CLASSIFY_VF(CASE111, CASE110, CASE101, CASE011, CASE001, CASE010, CASE100)\
    auto type = GetVFDistanceType(vertex, face1, face2, face3);\
    const double lambda1 = 1 - type._coef1 - type._coef2,\
                 lambda2 = type._coef2,\
                 lambda3 = type._coef1;\
    const int cnt = (lambda1 >= 0) + (lambda2 >= 0) + (lambda3 >= 0);\
    const int state = ((lambda3 >= 0) << 2) + ((lambda2 >= 0) << 1) + (lambda1 >= 0);\
    \
    switch (state) {\
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

double GetVFDistance(const Vector3d& vertex, const Vector3d& face1, const Vector3d& face2, const Vector3d& face3) {
    double d;
    CLASSIFY_VF(
        d = GetVPDistance(vertex, face1, face2, face3, lambda3, lambda2);,
        d = GetVLDistance(vertex, face2, face3);,
        d = GetVLDistance(vertex, face1, face3);,
        d = GetVLDistance(vertex, face1, face2);,
        d = GetVVDistance(vertex, face1);,
        d = GetVVDistance(vertex, face2);,
        d = GetVVDistance(vertex, face3);
    )
    return d;
}


const std::array<int, 9> index110 = {0, 1, 2, 6, 7, 8, 9, 10, 11};
const std::array<int, 9> index101 = {0, 1, 2, 3, 4, 5, 9, 10, 11};
const std::array<int, 9> index011 = {0, 1, 2, 3, 4, 5, 6, 7, 8};
const std::array<int, 6> index001 = {0, 1, 2, 3, 4, 5};
const std::array<int, 6> index010 = {0, 1, 2, 6, 7, 8};
const std::array<int, 6> index100 = {0, 1, 2, 9, 10, 11};

Vector12d GetVFDistanceGradient(const Vector3d &vertex, const Vector3d &face1, const Vector3d &face2, const Vector3d &face3) {
    Vector12d pdpx = Vector12d::Zero();
    CLASSIFY_VF(
        pdpx = GetVPDistanceGradient(vertex, face1, face2, face3, lambda3, lambda2);,
        pdpx(index110) = GetVLDistanceGradient(vertex, face2, face3);,
        pdpx(index101) = GetVLDistanceGradient(vertex, face1, face3);,
        pdpx(index011) = GetVLDistanceGradient(vertex, face1, face2);,
        pdpx(index001) = GetVVDistanceGradient(vertex, face1);,
        pdpx(index010) = GetVVDistanceGradient(vertex, face2);,
        pdpx(index100) = GetVVDistanceGradient(vertex, face3);
    )
    return pdpx;
}

Matrix12d GetVFDistanceHessian(const Vector3d &vertex, const Vector3d &face1, const Vector3d &face2, const Vector3d &face3) {
    Matrix12d p2dpx2 = Matrix12d::Zero();
    CLASSIFY_VF(
        p2dpx2 = GetVPDistanceHessian(vertex, face1, face2, face3, lambda3, lambda2);,
        p2dpx2(index110, index110) = GetVLDistanceHessian(vertex, face2, face3);,
        p2dpx2(index101, index101) = GetVLDistanceHessian(vertex, face1, face3);,
        p2dpx2(index011, index011) = GetVLDistanceHessian(vertex, face1, face2);,
        p2dpx2(index001, index001) = GetVVDistanceHessian(vertex, face1);,
        p2dpx2(index010, index010) = GetVVDistanceHessian(vertex, face2);,
        p2dpx2(index100, index100) = GetVVDistanceHessian(vertex, face3);
    )
    return p2dpx2;
}

#define CLASSIFY_EE(CASE1111, CASE1110, CASE1101, CASE1011, CASE0111, CASE1010, CASE1001, CASE0110, CASE0101) \
    auto type = GetEEDistanceType(edge11, edge12, edge21, edge22);\
    const double lambda1 = type._coef1, lambda2 = type._coef2;\
    const int state = ((lambda2 <= 1) << 3) + ((0 <= lambda2) << 2) + ((lambda1 <= 1) << 1) + (0 <= lambda1);\
    \
    switch(state) {\
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

double GetEEDistance(const Vector3d& edge11, const Vector3d& edge12, const Vector3d& edge21, const Vector3d& edge22) {
    double d;
    CLASSIFY_EE(
        d = GetLLDistance(edge11, edge12, edge21, edge22, lambda1, lambda2);,
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

const std::array<int, 9> index1110 = {0, 1, 2, 6, 7, 8, 9, 10, 11};
const std::array<int, 9> index1101 = {3, 4, 5, 6, 7, 8, 9, 10, 11};
const std::array<int, 9> index1011 = {6, 7, 8, 0, 1, 2, 3, 4, 5};
const std::array<int, 9> index0111 = {9, 10, 11, 0, 1, 2, 3, 4, 5};
const std::array<int, 6> index1010 = {0, 1, 2, 6, 7, 8};
const std::array<int, 6> index1001 = {3, 4, 5, 6, 7, 8};
const std::array<int, 6> index0110 = {0, 1, 2, 9, 10, 11};
const std::array<int, 6> index0101 = {3, 4, 5, 9, 10, 11};

Vector12d GetEEDistanceGradient(const Vector3d &edge11, const Vector3d &edge12, const Vector3d &edge21, const Vector3d &edge22) {
    Vector12d pdpx = Vector12d::Zero();
    CLASSIFY_EE(
        pdpx            = GetLLDistanceGradient(edge11, edge12, edge21, edge22, lambda1, lambda2);,
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

Matrix12d GetEEDistanceHessian(const Vector3d &edge11, const Vector3d &edge12, const Vector3d &edge21, const Vector3d &edge22) {
    Matrix12d p2dpx2 = Matrix12d::Zero();
    CLASSIFY_EE(
        p2dpx2                       = GetLLDistanceHessian(edge11, edge12, edge21, edge22, lambda1, lambda2);,
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

