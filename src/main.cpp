
#include <libwebsockets.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <assert.h>

#include "spdlog/spdlog.h"
#include "spdlog/cfg/env.h"  
#include "spdlog/fmt/ostr.h" 
#include <memory>
#include "agent.h"
#include <chrono>
#include "../lib/mqtt/wrapper.h"

#include "../lib/ds/GenericHID.h"
#include "../lib/ds/XboxController.h"
#include "../lib/framework/TimedRobot.h"
#include "../lib/ds/EventLoop.h"
#include "../lib/ds/BooleanEvent.h"
#include "../lib/motor/MotorController.h"
#include <iostream>

using namespace spdlog;

static const auto SHOT_VELOCITY = 200;    //200_rpm;
static const auto TOLERANCE = 8;          // 8_rpm;
static const auto KICKER_THRESHOLD = 15;  // 15_mm;

class Robot : public TimedRobot {
public:
    void AutonomousPeriodic() override {
        DriveWithJoystick(false);
    }

    void TeleopPeriodic() override { DriveWithJoystick(true); }


    void DriveWithJoystick(bool fieldRelative) {
        std::cout <<"Robot DriveWithJoystick" << std::endl;
        const auto xSpeed = m_controller.GetLeftY();
        const auto ySpeed = m_controller.GetLeftX();
        const auto rot = m_controller.GetRightX();
/*
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

        BooleanEvent intakeButton{ &m_loop, [&joystick = m_joystick] { return joystick.GetRawButton(2); }};
        BooleanEvent shootTrigger{ &m_loop, [&joystick = m_joystick] { return joystick.GetRawButtonPressed(2);}};
    }

    void RobotPeriodic() override { m_loop.Poll(); }

private:
    EventLoop m_loop{};
    XboxController m_controller{0};
    GenericHID m_joystick{0};
};

int main() {
    return StartRobot<Robot>();
}