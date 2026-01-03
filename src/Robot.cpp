#include "Robot.h"
using namespace spdlog;

void Robot::RobotInit() {
}

void Robot::AutonomousInit() {
    SubsystemBase::RunAllAutonomousInit();
    //m_autonChooser.AwaitAutonomous();
}

/**
 * Initialization code for teleop mode should go here.
 */
void Robot::TeleopInit() {
    //m_autonChooser.CancelAutonomous();
    SubsystemBase::RunAllTeleopInit();
}

/**
 * Periodic code for all modes should go here.
 */
void Robot::RobotPeriodic() {
    m_loop.Poll();
}

void Robot::AutonomousPeriodic() {
    SubsystemBase::RunAllAutonomousPeriodic();
    DriveWithJoystick(false);
}

void Robot::TeleopPeriodic() {
    SubsystemBase::RunAllRobotPeriodic();
    spdlog::info("Robot::TeleopPeriodic");
    DriveWithJoystick(true);
}

void Robot::DriveWithJoystick(__attribute__((unused)) bool fieldRelative) {
    std::cout << "Robot DriveWithJoystick" << std::endl;
    const auto xSpeed = m_controller.GetLeftY();
    const auto ySpeed = m_controller.GetLeftX();
    const auto rot = m_controller.GetRightX();
    /*
    https://github.com/wpilibsuite/allwpilib/blob/d32e60233fe516e8f67e0e94d3de87615e09f00f/wpilibcExamples/src/main/cpp/examples/EventLoop/cpp/Robot.cpp#L12
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        const auto xSpeed = -m_xspeedLimiter.Calculate(
                frc::ApplyDeadband(m_controller.GetLeftY(), 0.02)) *
                            Drivetrain::kMaxSpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        const auto ySpeed = -m_yspeedLimiter.Calculate(
                frc::ApplyDeadband(m_controller.GetLeftX(), 0.02)) *
                            Drivetrain::kMaxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        const auto rot = -m_rotLimiter.Calculate(
                frc::ApplyDeadband(m_controller.GetRightX(), 0.02)) *
                         Drivetrain::kMaxAngularSpeed;

        m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative, GetPeriod());
*/

    BooleanEvent intakeButton{&m_loop, [&joystick = m_joystick] { return joystick.GetRawButton(2); }};
    BooleanEvent shootTrigger{&m_loop, [&joystick = m_joystick] { return joystick.GetRawButtonPressed(2); }};
}

int main() {
    return StartRobot<Robot>();
}
