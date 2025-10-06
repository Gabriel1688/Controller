
#include <libwebsockets.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <assert.h>
#include <iostream>
#include "spdlog/spdlog.h"
#include "spdlog/cfg/env.h"  
#include "spdlog/fmt/ostr.h" 
#include <memory>
#include <chrono>
#include "ds/GenericHID.h"
#include "ds/XboxController.h"
#include "ds/EventLoop.h"
#include "ds/BooleanEvent.h"
#include "mqtt/wrapper.h"
#include "robot/TimedRobot.h"
#include "Constants.h"
#include "motor/CAN.h"

using namespace spdlog;

static const auto SHOT_VELOCITY = 200;    //200_rpm;
static const auto TOLERANCE = 8;          // 8_rpm;
static const auto KICKER_THRESHOLD = 15;  // 15_mm;

class Robot : public TimedRobot {
public:
    void AutonomousPeriodic() override {
        DriveWithJoystick(false);
    }

    void RobotInit() {
        std::shared_ptr<CAN> can = std::make_shared<CAN>(DriveConstants::kFrontLeftDrivingCanId, HAL_CAN_Man_Dummy, HAL_CAN_Dev_karmController);
        uint8_t data[]={0xFF,0xFF,0xFF,0xFF, 0xFF,0xFF,0xFF,0xFC};
        uint8_t apiId = 0x11;
        can->WritePacket(data,sizeof(data)/sizeof(uint8_t),apiId);
    }

    void TeleopPeriodic() override { DriveWithJoystick(true); }


    void DriveWithJoystick(__attribute__((unused)) bool fieldRelative) {
        //std::cout <<"Robot DriveWithJoystick" << std::endl;
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