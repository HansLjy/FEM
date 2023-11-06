#include "PointEdgeCcd.h"

namespace IPC {

    bool check_overlap(
        const Vector3d& x0,
        const Vector3d& x1,
        const Vector3d& x2,
        const Vector3d& d0,
        const Vector3d& d1,
        const Vector3d& d2,
        double root)
    {
        Vector3d p0 = x0 + d0 * root;
        Vector3d e0 = x1 + d1 * root;
        Vector3d e1 = x2 + d2 * root;
        Vector3d e = e1 - e0;
        double ratio = e.dot(p0 - e0) / e.squaredNorm();
        return 0 <= ratio && ratio <= 1;
    }
}

namespace IPC {
    bool point_edge_cd_broadphase(
        const Vector3d& x0,
        const Vector3d& x1,
        const Vector3d& x2,
        double dist)
    {
        const Eigen::Array<double, 3, 1> max_e = x1.array().max(x2.array());
        const Eigen::Array<double, 3, 1> min_e = x1.array().min(x2.array());
        if ((x0.array() - max_e > dist).any() || (min_e - x0.array() > dist).any()) {
            return false;
        }
        else {
            return true;
        }
    }

    bool point_edge_ccd_broadphase(
        const Vector3d& p,
        const Vector3d& e0,
        const Vector3d& e1,
        const Vector3d& dp,
        const Vector3d& de0,
        const Vector3d& de1,
        const double dist)
    {
        const Eigen::Array<double, 3, 1> max_p = p.array().max((p + dp).array());
        const Eigen::Array<double, 3, 1> min_p = p.array().min((p + dp).array());
        const Eigen::Array<double, 3, 1> max_e = e0.array().max(e1.array()).max((e0 + de0).array()).max((e1 + de1).array());
        const Eigen::Array<double, 3, 1> min_e = e0.array().min(e1.array()).min((e0 + de0).array()).min((e1 + de1).array());
        // check overlap of bounding box
        if ((min_p - max_e > dist).any() || (min_e - max_p > dist).any()) {
            return false;
        }
        else {
            return true;
        }
    }

    // double point_edge_ccd(
    //     const Vector<double, 2>& x0,
    //     const Vector<double, 2>& x1,
    //     const Vector<double, 2>& x2,
    //     const Vector<double, 2>& d0,
    //     const Vector<double, 2>& d1,
    //     const Vector<double, 2>& d2,
    //     double eta)
    // {
    //     double toc = 1;
    //     double a = d0[0] * (d2[1] - d1[1]) + d0[1] * (d1[0] - d2[0]) + d2[0] * d1[1] - d2[1] * d1[0];
    //     double b = x0[0] * (d2[1] - d1[1]) + d0[0] * (x2[1] - x1[1]) + d0[1] * (x1[0] - x2[0]) + x0[1] * (d1[0] - d2[0]) + d1[1] * x2[0] + d2[0] * x1[1] - d1[0] * x2[1] - d2[1] * x1[0];
    //     double c = x0[0] * (x2[1] - x1[1]) + x0[1] * (x1[0] - x2[0]) + x2[0] * x1[1] - x2[1] * x1[0];

    //     if (a == 0) {
    //         if (b == 0) {
    //             // parallel motion, only need to handle colinear case
    //             if (c == 0) {
    //                 // colinear PP CCD
    //                 if ((x0 - x1).dot(d0 - d1) < 0) {
    //                     double root = std::sqrt((x0 - x1).squaredNorm() / (d0 - d1).squaredNorm());
    //                     if (root > 0 && root <= 1)
    //                         toc = std::min(toc, root * (1 - eta));
    //                 }
    //                 if ((x0 - x2).dot(d0 - d2) < 0) {
    //                     double root = std::sqrt((x0 - x2).squaredNorm() / (d0 - d2).squaredNorm());
    //                     if (root > 0 && root <= 1)
    //                         toc = std::min(toc, root * (1 - eta));
    //                 }
    //             }
    //         }
    //         else {
    //             double root = -c / b;
    //             if (root > 0 && root <= 1 && check_overlap(x0, x1, x2, d0, d1, d2, root))
    //                 toc = std::min(toc, root * (1 - eta));
    //         }
    //     }
    //     else {
    //         double delta = b * b - 4 * a * c;
    //         if (delta == 0) {
    //             double root = -b / (2 * a);
    //             if (root > 0 && root <= 1 && check_overlap(x0, x1, x2, d0, d1, d2, root))
    //                 toc = std::min(toc, root * (1 - eta));
    //         }
    //         else if (delta > 0) {
    //             // accurate expression differs in b's sign
    //             if (b > 0) {
    //                 double root = (-b - std::sqrt(delta)) / (2 * a);
    //                 if (root > 0 && root <= 1 && check_overlap(x0, x1, x2, d0, d1, d2, root))
    //                     toc = std::min(toc, root * (1 - eta));
    //                 root = 2 * c / (-b - std::sqrt(delta));
    //                 if (root > 0 && root <= 1 && check_overlap(x0, x1, x2, d0, d1, d2, root))
    //                     toc = std::min(toc, root * (1 - eta));
    //             }
    //             else {
    //                 double root = 2 * c / (-b + std::sqrt(delta));
    //                 if (root > 0 && root <= 1 && check_overlap(x0, x1, x2, d0, d1, d2, root))
    //                     toc = std::min(toc, root * (1 - eta));
    //                 root = (-b + std::sqrt(delta)) / (2 * a);
    //                 if (root > 0 && root <= 1 && check_overlap(x0, x1, x2, d0, d1, d2, root))
    //                     toc = std::min(toc, root * (1 - eta));
    //             }
    //         }
    //     }
    //     return toc;
    // }
}