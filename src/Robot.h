#pragma  once

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
#include "Constants.h"
#include "ds/GenericHID.h"
#include "ds/XboxController.h"
#include "ds/EventLoop.h"
#include "ds/BooleanEvent.h"
#include "robot/TimedRobot.h"
#include "subsystem/Arm.h"
#include "subsystem/Gripper.h"

using namespace spdlog;

class Robot : public TimedRobot {
public:
    void RobotInit() override;
    void DriveWithJoystick(bool fieldRelative) ;

    /**
     * Periodic code for all modes should go here.
     */
    void RobotPeriodic() override;

    /**
     * Initialization code for autonomous mode should go here.
     */
    void AutonomousInit() override ;

    /**
     * Initialization code for teleop mode should go here.
     */
    void TeleopInit()  override ;

    /**
     * Periodic code for autonomous mode should go here.
     */
    void AutonomousPeriodic() override;

    /**
     * Periodic code for teleop mode should go here.
     */
    void TeleopPeriodic() override ;
    Robot()= default;
    ~Robot() override = default;

private:
    EventLoop m_loop{};
    XboxController m_controller{0};
    GenericHID m_joystick{0};

    /// Arm subsystem.
    Arm arm;

    /// Gripper subsystem.
    Gripper gripper;

    /// Perception subsystem.
    //Gripper gripper;
};

