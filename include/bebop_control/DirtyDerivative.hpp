#ifndef BEBOP_CONTROL_DIRTYDERIVATIVE_HPP
#define BEBOP_CONTROL_DIRTYDERIVATIVE_HPP

#include <Eigen/Dense>

namespace bebop_control {

class DirtyDerivative {
public:
    // N = 1/tau (cutoff frequency in rad/s)
    DirtyDerivative(double N, double dt);

    void propagate(const Eigen::Vector3d& x_meas);

    Eigen::Vector3d get_velocity_estimate() const;

private:
    double a1_, a2_;
    bool initialized_;
    Eigen::Vector3d x_prev_;
    Eigen::Vector3d v_est_;
};

}

#endif