#include <Eigen/Core>
#include <algorithm>

#include "Arm.h"
#include "ds/DriverStation.h"
#include "robot/RobotBase.h"
#include "motor/CtrlStepMotor.h"
#include "ControlledSubsystemBase.h"

Arm::Arm()
{
    // Reset the pose estimate to the field's bottom-left corner with the turret
    // facing in the target's general direction. This is relatively close to the
    // robot's testing configuration, so the turret won't hit the soft limits.
    Reset(Pose2d{0.0, 0.0, 0.0});
}
//
//Pose2d Arm::GetReferencePose() const {
//    const auto& x = m_controller.GetReferences();
//    return Pose2d{
//            float {x(ArmController::State::kX)},
//            float {x(ArmController::State::kY)},
//            float {x(ArmController::State::kHeading)}};
//}

//Pose2d Arm::GetPose() const { return m_observer.GetPose(); }

void Arm::Reset(const Pose2d& initialPose) {
}

void Arm::ControllerPeriodic() {

}

void Arm::RobotPeriodic() {

}

const Eigen::Vector<double, 2>& Arm::GetInputs() const {
    return m_controller.GetInputs();
}

void Arm::DisabledInit() {
    SetBrakeMode();
    Disable();
}

void Arm::AutonomousInit() {
    SetBrakeMode();
    //SetTurningTolerance(0.25);
    Enable();
}

void Arm::TeleopInit() {
    SetBrakeMode();

    // If the robot was disabled while still following a trajectory in
    // autonomous, it will continue to do so in teleop. This aborts any
    // trajectories so teleop driving can occur.
    // m_controller.AbortTrajectories();

    // If the robot was disabled while still turning in place in
    // autonomous, it will continue to do so in teleop. This aborts any
    // turning action so teleop driving can occur.
    // AbortTurnInPlace();

    Enable();
}

void Arm::TeleopPeriodic() {
/*
    using Input = ArmController::Input;

    static frc::Joystick driveStick1{HWConfig::kDriveStick1Port};
    static frc::Joystick driveStick2{HWConfig::kDriveStick2Port};

    double y = frc::ApplyDeadband(-driveStick1.GetY(), Constants::kJoystickDeadband);
    double x = frc::ApplyDeadband(driveStick2.GetX(), Constants::kJoystickDeadband);

    if (driveStick2.GetRawButton(2)) {
        x *= 0.4;
    }

    auto [left, right] = frc::DifferentialDrive::CurvatureDriveIK(
            y, x, driveStick2.GetRawButton(2));

    // Implicit model following
    // TODO: Velocities need filtering
    Eigen::Vector<double, 2> u =
            m_imf.Calculate(Eigen::Vector<double, 2>{GetLeftVelocity().value(),
                                                     GetRightVelocity().value()},
                            Eigen::Vector<double, 2>{left * 12.0, right * 12.0});

    if (!IsVisionAiming()) {
        m_leftGrbx.SetVoltage(units::volt_t{u(Input::kLeftVoltage)});
        m_rightGrbx.SetVoltage(units::volt_t{u(Input::kRightVoltage)});
    }

    m_headingGoalEntry.SetBoolean(AtHeading());
    m_hasHeadingGoalEntry.SetBoolean(HasHeadingGoal());

    m_yawControllerEntry.SetDouble(GetVisionYaw().value());
    m_rangeControllerEntry.SetDouble(m_controller.GetVisionRange().value());
*/
}

void Arm::SetBrakeMode() {
}

void Arm::SetCoastMode() {
}
