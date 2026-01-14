#include "bebop_control/DirtyDerivative.hpp"
#include <cmath>

namespace bebop_control {

DirtyDerivative::DirtyDerivative(double cutoff_freq_hz) 
    : initialized_(false) 
{
    x_prev_.setZero();
    v_est_.setZero();

    tau_ = 1.0 / (2.0 * M_PI * cutoff_freq_hz);
}

void DirtyDerivative::propagate_filter(const Eigen::Vector3d& x_meas, double dt) 
{
    if (!x_meas.allFinite()) return;
    
    if (!initialized_) {
        x_prev_ = x_meas;
        v_est_.setZero();
        initialized_ = true;
        return;
    }

    // Tustin Discretization
    // G(s) = s / (tau*s + 1)
    double a1 = (2.0 * tau_ - dt) / (2.0 * tau_ + dt);
    double a2 = 2.0 / (2.0 * tau_ + dt);

    v_est_ = a1 * v_est_ + a2 * (x_meas - x_prev_);
    x_prev_ = x_meas;
}

const Eigen::Vector3d& DirtyDerivative::get_velocity_estimate() const 
{
    return v_est_;
}

const Eigen::Vector3d& DirtyDerivative::get_position_estimate() const 
{
    return x_prev_;
}

} // namespace bebop_control
