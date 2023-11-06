#include "EdgeEdgeCcd.h"

#include "EdgeEdgeDistance.h"
#include "PointPointDistance.h"
#include "PointEdgeDistance.h"
#include "DistanceType.h"

namespace IPC {

    double edge_edge_distance_unclassified(
        const Vector3d& ea0,
        const Vector3d& ea1,
        const Vector3d& eb0,
        const Vector3d& eb1
    ) {
        switch (edge_edge_distance_type(ea0, ea1, eb0, eb1)) {
        case 0:
            return point_point_distance(ea0, eb0);
        case 1:
            return point_point_distance(ea0, eb1);
        case 2:
            return point_edge_distance(ea0, eb0, eb1);
        case 3:
            return point_point_distance(ea1, eb0);
        case 4:
            return point_point_distance(ea1, eb1);
        case 5:
            return point_edge_distance(ea1, eb0, eb1);
        case 6:
            return point_edge_distance(eb0, ea0, ea1);
        case 7:
            return point_edge_distance(eb1, ea0, ea1);
        case 8:
            return edge_edge_distance(ea0, ea1, eb0, eb1);
        default:
            return std::numeric_limits<double>::max();
        }
    }

    bool edge_edge_cd_broadphase(
        const Vector3d& ea0,
        const Vector3d& ea1,
        const Vector3d& eb0,
        const Vector3d& eb1,
        double dist
    ) {
        auto max_a = ea0.array().max(ea1.array());
        auto min_a = ea0.array().min(ea1.array());
        auto max_b = eb0.array().max(eb1.array());
        auto min_b = eb0.array().min(eb1.array());
        if ((min_a - max_b > dist).any() || (min_b - max_a > dist).any())
            return false;
        else
            return true;
    }

    bool edge_edge_ccd_broadphase(
        const Vector3d& ea0,
        const Vector3d& ea1,
        const Vector3d& eb0,
        const Vector3d& eb1,
        const Vector3d& dea0,
        const Vector3d& dea1,
        const Vector3d& deb0,
        const Vector3d& deb1,
        double dist
    ) {
        auto max_a = ea0.array().max(ea1.array()).max((ea0 + dea0).array()).max((ea1 + dea1).array());
        auto min_a = ea0.array().min(ea1.array()).min((ea0 + dea0).array()).min((ea1 + dea1).array());
        auto max_b = eb0.array().max(eb1.array()).max((eb0 + deb0).array()).max((eb1 + deb1).array());
        auto min_b = eb0.array().min(eb1.array()).min((eb0 + deb0).array()).min((eb1 + deb1).array());
        if ((min_a - max_b > dist).any() || (min_b - max_a > dist).any())
            return false;
        else
            return true;
    }

    double edge_edge_ccd(
        const Vector3d& _ea0,
        const Vector3d& _ea1,
        const Vector3d& _eb0,
        const Vector3d& _eb1,
        const Vector3d& _dea0,
        const Vector3d& _dea1,
        const Vector3d& _deb0,
        const Vector3d& _deb1,
        double eta, double thickness
    ) {
        Vector3d ea0 = _ea0, ea1 = _ea1, eb0 = _eb0, eb1 = _eb1, dea0 = _dea0, dea1 = _dea1, deb0 = _deb0, deb1 = _deb1;
        Vector3d mov = (dea0 + dea1 + deb0 + deb1) / 4;
        dea0 -= mov;
        dea1 -= mov;
        deb0 -= mov;
        deb1 -= mov;
        double max_disp_mag = std::sqrt(std::max(dea0.squaredNorm(), dea1.squaredNorm())) + std::sqrt(std::max(deb0.squaredNorm(), deb1.squaredNorm()));
        if (max_disp_mag == 0)
            return 2;

        double dist2_cur = edge_edge_distance_unclassified(ea0, ea1, eb0, eb1);

        double dFunc = dist2_cur - thickness * thickness;
        if (dFunc <= 0) {
            // since we ensured other place that all dist smaller than dHat are positive,
            // this must be some far away nearly parallel edges
            std::vector<double> dists{ (ea0 - eb0).squaredNorm(), (ea0 - eb1).squaredNorm(), (ea1 - eb0).squaredNorm(), (ea1 - eb1).squaredNorm() };
            dist2_cur = *std::min_element(dists.begin(), dists.end());
            dFunc = dist2_cur - thickness * thickness;
        }
        double dist_cur = std::sqrt(dist2_cur);
        double gap = eta * dFunc / (dist_cur + thickness);
        double toc = 0.0;

        int i = 0;
        while (true) {
            double toc_lower_bound = (1 - eta) * dFunc / ((dist_cur + thickness) * max_disp_mag);
            ea0 += toc_lower_bound * dea0;
            ea1 += toc_lower_bound * dea1;
            eb0 += toc_lower_bound * deb0;
            eb1 += toc_lower_bound * deb1;
            dist2_cur = edge_edge_distance_unclassified(ea0, ea1, eb0, eb1);
            dFunc = dist2_cur - thickness * thickness;
            if (dFunc <= 0) {
                // since we ensured other place that all dist smaller than dHat are positive,
                // this must be some far away nearly parallel edges
                std::vector<double> dists{ (ea0 - eb0).squaredNorm(), (ea0 - eb1).squaredNorm(), (ea1 - eb0).squaredNorm(), (ea1 - eb1).squaredNorm() };
                dist2_cur = *std::min_element(dists.begin(), dists.end());
                dFunc = dist2_cur - thickness * thickness;
            }
            dist_cur = std::sqrt(dist2_cur);
            if (toc && (dFunc / (dist_cur + thickness) < gap)) {
                break;
            }
            toc += toc_lower_bound;
            if (toc > 1.0)
                return 2;
        }
        return toc;
    }
}