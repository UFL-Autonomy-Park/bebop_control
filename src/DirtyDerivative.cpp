#include "bebop_control/DirtyDerivative.hpp"

namespace bebop_control {

DirtyDerivative::DirtyDerivative(double N, double dt) 
    : initialized_(false) {
    
    x_prev_.setZero();
    v_est_.setZero();

    // Tustin Discretization
    // G(s) = s / (tau*s + 1), tau = 1/N
    double tau = 1.0 / N;
    a1_ = (2.0 * tau - dt) / (2.0 * tau + dt);
    a2_ = 2.0 / (2.0 * tau + dt);
}

void DirtyDerivative::propagate(const Eigen::Vector3d& x_meas) {
    // Make sure measurement is valid
    if (!x_meas.allFinite()) return;

    if (!initialized_) {
        x_prev_ = x_meas;
        v_est_.setZero();
        initialized_ = true;
        return;
    }

    v_est_ = a1_ * v_est_ + a2_ * (x_meas - x_prev_);
    x_prev_ = x_meas;
}

Eigen::Vector3d DirtyDerivative::get_velocity_estimate() const {
    return v_est_;
}

}