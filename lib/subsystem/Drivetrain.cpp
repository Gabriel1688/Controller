#include <Eigen/Core>
#include <algorithm>

#include "Drivetrain.h"
#include "ds/DriverStation.h"
#include "robot/RobotBase.h"
#include "motor/CtrlStepMotor.h"
#include "ControlledSubsystemBase.h"

Drivetrain::Drivetrain()
{
    // Reset the pose estimate to the field's bottom-left corner with the turret
    // facing in the target's general direction. This is relatively close to the
    // robot's testing configuration, so the turret won't hit the soft limits.
    Reset(Pose2d{0.0, 0.0, 0.0});
}
//
//Pose2d Drivetrain::GetReferencePose() const {
//    const auto& x = m_controller.GetReferences();
//    return Pose2d{
//            float {x(DrivetrainController::State::kX)},
//            float {x(DrivetrainController::State::kY)},
//            float {x(DrivetrainController::State::kHeading)}};
//}

//Pose2d Drivetrain::GetPose() const { return m_observer.GetPose(); }

void Drivetrain::Reset(const Pose2d& initialPose) {
}

void Drivetrain::ControllerPeriodic() {

}

void Drivetrain::RobotPeriodic() {

}

const Eigen::Vector<double, 2>& Drivetrain::GetInputs() const {
    return m_controller.GetInputs();
}

void Drivetrain::DisabledInit() {
    SetBrakeMode();
    Disable();
}

void Drivetrain::AutonomousInit() {
    SetBrakeMode();
    //SetTurningTolerance(0.25);
    Enable();
}

void Drivetrain::TeleopInit() {
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

void Drivetrain::TestInit() {
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

void Drivetrain::TeleopPeriodic() {
/*
    using Input = DrivetrainController::Input;

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

void Drivetrain::TestPeriodic() {

}

void Drivetrain::SetBrakeMode() {
}

void Drivetrain::SetCoastMode() {
}
