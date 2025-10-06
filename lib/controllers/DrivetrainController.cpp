#include <cmath>
#include <algorithm>
#include "DrivetrainController.h"
#include "subsystem/Pose2d.h"

DrivetrainController::DrivetrainController() {

}


void DrivetrainController::Reset(const Pose2d& initialPose) {
}

Eigen::Vector<double, 2> DrivetrainController::Calculate(const Eigen::Vector<double, 7>& x)
{
    m_u = Eigen::Vector<double, 2>::Zero();

    return m_u;
}
Eigen::Vector<double, 7> DrivetrainController::Dynamics(const Eigen::Vector<double, 7>& x, const Eigen::Vector<double, 2>& u)
{
    Eigen::Vector<double, 7> xdot;
    return xdot;
}

