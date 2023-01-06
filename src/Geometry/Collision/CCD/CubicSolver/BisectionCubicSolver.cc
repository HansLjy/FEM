//
// Created by hansljy on 11/23/22.
//

#include "BisectionCubicSolver.h"
#include "cmath"

double BisectionCubicSolver::Solve(double A, double B, double C, double D, double l, double r) {
    if (A < 0) {
        A = -A; B = -B; C = -C; D = -D;
    }

    auto f = [&](double x) {
        return (((A * x) + B) * x + C) * x + D;
    };

    const double FAIL = FAIL;

    if (std::abs(A) < _tolerance) {
        if (std::abs(B) < _tolerance) {
            if (std::abs(C) < _tolerance) {
                return std::abs(D) < _tolerance ? l : FAIL;
            } else {
                double root = - D / C;
                return l <= root && root <= r ? root : FAIL;
            }
        } else {
            const double delta = C * C - 4 * B * D;
            if (delta < 0) {
                return FAIL;
            } else {
                const double delta_rt = std::sqrt(delta);
                double root1 = (-C - delta_rt) / (2 * B);
                double root2 = (-C + delta_rt) / (2 * B);
                if (root1 > root2) {
                    std::swap(root1, root2);
                }
                if (root1 >= l && root1 <= r) {
                    return root1;
                } else if (root2 >= l && root2 <= r) {
                    return root2;
                } else {
                    return FAIL;
                }
            }
        }
    } else {
        // solve quadratic equation of maxima and minimum
        double A2 = 3 * A,
               B2 = 2 * B;
        double delta = B2 * B2 - 4 * A2 * C;
        if (delta <= 0) {
            // monotonically increasing
            if (f(l) > 0) {
                return FAIL;
            } else {
                return FindRoot(f, l, r);
            }
        } else {
            const double delta_rt = std::sqrt(delta);
            const double maxima_x = (-B2 - delta_rt) / (2 * A2);
            const double minima_x = (-B2 + delta_rt) / (2 * A2);

            if (r <= maxima_x || l >= minima_x || (maxima_x <= l && r <= minima_x)) {
                if (f(l) * f(r) > 0) {
                    return FAIL;
                } else {
                    return FindRoot(f, l, r);
                }
            } else {
                if (maxima_x <= r && r <= minima_x) {
                    if (f(l) <= 0) {
                        if (f(maxima_x) >= 0) {
                            return FindRoot(f, l, maxima_x);
                        } else {
                            return FAIL;
                        }
                    } else {
                        if (f(r) <= 0) {
                            return FindRoot(f, maxima_x, r);
                        } else {
                            return FAIL;
                        }
                    }
                }

                if (maxima_x <= l && l <= minima_x) {
                    if (f(l) >= 0) {
                        if (f(minima_x) <= 0) {
                            return FindRoot(f, l, minima_x);
                        } else {
                            return FAIL;
                        }
                    } else {
                        if (f(r) >= 0) {
                            return FindRoot(f, minima_x, r);
                        } else {
                            return FAIL;
                        }
                    }
                }

                if (f(l) <= 0) {
                    if (f(maxima_x) >= 0) {
                        return FindRoot(f, l, maxima_x);
                    } else if (f(r) >= 0) {
                        return FindRoot(f, minima_x, r);
                    } else {
                        return FAIL;
                    }
                } else if (f(minima_x) <= 0) {
                    return FindRoot(f, maxima_x, minima_x);
                } else {
                    return FAIL;
                }
            }
        }
    }
}

double BisectionCubicSolver::FindRoot(const std::function<double(double)>& f, double l, double r) {
    double fl = f(l);
    double fr = f(r);
    while (r - l > _tolerance) {
        double mid = (l + r) / 2;
        double fmid = f(mid);
        if (fmid * fl > 0) {
            l = mid;
            fl = fmid;
        } else {
            r = mid;
            fr = fmid;
        }
    }
    return l;
}