#include <cmath>
#include <algorithm>
#include "ArmController.h"
#include "subsystem/Pose2d.h"

ArmController::ArmController() {

}

void ArmController::Reset(const Pose2d& initialPose) {
}

Eigen::Vector<double, 2> ArmController::Calculate(const Eigen::Vector<double, 7>& x)
{
    m_u = Eigen::Vector<double, 2>::Zero();

    return m_u;
}
Eigen::Vector<double, 7> ArmController::Dynamics(const Eigen::Vector<double, 7>& x, const Eigen::Vector<double, 2>& u)
{
    Eigen::Vector<double, 7> xdot;
    return xdot;
}

