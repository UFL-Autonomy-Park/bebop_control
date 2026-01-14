#ifndef BEBOP_CONTROL_DIRTYDERIVATIVE_HPP
#define BEBOP_CONTROL_DIRTYDERIVATIVE_HPP

#include <Eigen/Dense>

namespace bebop_control {

class DirtyDerivative {
public:
    explicit DirtyDerivative(double cutoff_freq_hz);

    void propagate_filter(const Eigen::Vector3d& x_meas, double dt);

    const Eigen::Vector3d& get_velocity_estimate() const;
    const Eigen::Vector3d& get_position_estimate() const;

private:
    double tau_;
    bool initialized_;
    Eigen::Vector3d x_prev_;
    Eigen::Vector3d v_est_;
};

} // namespace bebop_control

#endif