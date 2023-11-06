#include "PointTriangleCcd.h"
#include "PointPointDistance.h"
#include "PointEdgeDistance.h"
#include "PointTriangleDistance.h"
#include "DistanceType.h"

namespace IPC {

    double point_triangle_distance_unclassified(
        const Vector3d& p,
        const Vector3d& t0,
        const Vector3d& t1,
        const Vector3d& t2)
    {
        switch (point_triangle_distance_type(p, t0, t1, t2)) {
        case 0:
            return point_point_distance(p, t0);
        case 1:
            return point_point_distance(p, t1);
        case 2:
            return point_point_distance(p, t2);
        case 3:
            return point_edge_distance(p, t0, t1);
        case 4:
            return point_edge_distance(p, t1, t2);
        case 5:
            return point_edge_distance(p, t2, t0);
        case 6:
            return point_triangle_distance(p, t0, t1, t2);
        default:
            return std::numeric_limits<double>::max();
        }
    }

    bool point_triangle_cd_broadphase(
        const Vector3d& p,
        const Vector3d& t0,
        const Vector3d& t1,
        const Vector3d& t2,
        double dist
    ) {
        auto max_tri = t0.array().max(t1.array()).max(t2.array());
        auto min_tri = t0.array().min(t1.array()).min(t2.array());
        if ((p.array() - max_tri > dist).any() || (min_tri - p.array() > dist).any())
            return false;
        else
            return true;
    }

    bool point_triangle_ccd_broadphase(
        const Vector3d& p,
        const Vector3d& t0,
        const Vector3d& t1,
        const Vector3d& t2,
        const Vector3d& dp,
        const Vector3d& dt0,
        const Vector3d& dt1,
        const Vector3d& dt2,
        double dist
    ) {
        auto max_p = p.array().max((p + dp).array());
        auto min_p = p.array().min((p + dp).array());
        auto max_tri = t0.array().max(t1.array()).max(t2.array()).max((t0 + dt0).array()).max((t1 + dt1).array()).max((t2 + dt2).array());
        auto min_tri = t0.array().min(t1.array()).min(t2.array()).min((t0 + dt0).array()).min((t1 + dt1).array()).min((t2 + dt2).array());
        if ((min_p - max_tri > dist).any() || (min_tri - max_p > dist).any())
            return false;
        else
            return true;
    }

    double point_triangle_ccd(
        const Vector3d& _p,
        const Vector3d& _t0,
        const Vector3d& _t1,
        const Vector3d& _t2,
        const Vector3d& _dp,
        const Vector3d& _dt0,
        const Vector3d& _dt1,
        const Vector3d& _dt2,
        double eta, double thickness
    ) {
        Vector3d p = _p, t0 = _t0, t1 = _t1, t2 = _t2, dp = _dp, dt0 = _dt0, dt1 = _dt1, dt2 = _dt2;
        Vector3d mov = (dt0 + dt1 + dt2 + dp) / 4;
        dt0 -= mov;
        dt1 -= mov;
        dt2 -= mov;
        dp -= mov;
        std::vector<double> disp_mag2_vec{ dt0.squaredNorm(), dt1.squaredNorm(), dt2.squaredNorm() };

        double max_disp_mag = dp.norm() + std::sqrt(*std::max_element(disp_mag2_vec.begin(), disp_mag2_vec.end()));
        if (max_disp_mag == 0)
            return 2;

        double dist2_cur = point_triangle_distance_unclassified(p, t0, t1, t2);

        double dist_cur = std::sqrt(dist2_cur);
        double gap = eta * (dist2_cur - thickness * thickness) / (dist_cur + thickness);
        double toc = 0.0;
        while (true) {
            double toc_lower_bound = (1 - eta) * (dist2_cur - thickness * thickness) / ((dist_cur + thickness) * max_disp_mag);
            //printf("cpu: %f, %f, %f, %f, %f, %f\n", toc_lower_bound, dist2_cur, eta, thickness, dist_cur, max_disp_mag);
            p += toc_lower_bound * dp;
            t0 += toc_lower_bound * dt0;
            t1 += toc_lower_bound * dt1;
            t2 += toc_lower_bound * dt2;
            dist2_cur = point_triangle_distance_unclassified(p, t0, t1, t2);
            dist_cur = std::sqrt(dist2_cur);
            if (toc && ((dist2_cur - thickness * thickness) / (dist_cur + thickness) < gap)) {
                break;
            }

            toc += toc_lower_bound;
            if (toc > 1.0) {
                return 2;
            }
        }
        return toc;
    }

    double point_triangle_ccd2(
        int tid,
        const Vector3d& _p,
        const Vector3d& _t0,
        const Vector3d& _t1,
        const Vector3d& _t2,
        const Vector3d& _dp,
        const Vector3d& _dt0,
        const Vector3d& _dt1,
        const Vector3d& _dt2,
        double eta, double thickness)
    {
        Vector3d p = _p, t0 = _t0, t1 = _t1, t2 = _t2, dp = _dp, dt0 = _dt0, dt1 = _dt1, dt2 = _dt2;
        Vector3d mov = (dt0 + dt1 + dt2 + dp) / 4;
        dt0 -= mov;
        dt1 -= mov;
        dt2 -= mov;
        dp -= mov;
        std::vector<double> disp_mag2_vec{ dt0.squaredNorm(), dt1.squaredNorm(), dt2.squaredNorm() };

        double sd = *std::max_element(disp_mag2_vec.begin(), disp_mag2_vec.end());
        double max_disp_mag = dp.norm() + std::sqrt(*std::max_element(disp_mag2_vec.begin(), disp_mag2_vec.end()));



        if (max_disp_mag == 0)
            return 1.0;

        double dist2_cur = point_triangle_distance_unclassified(p, t0, t1, t2);

        double dist_cur = std::sqrt(dist2_cur);
        double gap = eta * (dist2_cur - thickness * thickness) / (dist_cur + thickness);
        double toc = 0.0;

        //if (tid == 0)
        //{
        //	printf("cpu %f, %f, %f, %f\n", max_disp_mag, dist2_cur, dist_cur, gap);
        //}

        while (true) {
            double toc_lower_bound = (1 - eta) * (dist2_cur - thickness * thickness) / ((dist_cur + thickness) * max_disp_mag);
            //printf("cpu: %f, %f, %f, %f, %f, %f\n", toc_lower_bound, dist2_cur, eta, thickness, dist_cur, max_disp_mag);
            p += toc_lower_bound * dp;
            t0 += toc_lower_bound * dt0;
            t1 += toc_lower_bound * dt1;
            t2 += toc_lower_bound * dt2;
            dist2_cur = point_triangle_distance_unclassified(p, t0, t1, t2);
            dist_cur = std::sqrt(dist2_cur);
            if (toc && ((dist2_cur - thickness * thickness) / (dist_cur + thickness) < gap)) {
                break;
            }

            toc += toc_lower_bound;
            if (toc > 1.0) {
                return 1.0;
            }
        }
        return toc;
    }
}