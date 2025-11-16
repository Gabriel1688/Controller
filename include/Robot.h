#pragma once

#include "../src/subsystems/Arm.h"
#include "../src/subsystems/Gripper.h"
#include "Constants.h"
#include "ds/BooleanEvent.h"
#include "ds/EventLoop.h"
#include "ds/GenericHID.h"
#include "ds/XboxController.h"
#include "robot/TimedRobot.h"
#include "spdlog/cfg/env.h"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"
#include <assert.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <pthread.h>
#include <signal.h>
#include <string.h>

using namespace spdlog;

class Robot : public TimedRobot {
public:
    void RobotInit() override;
    void DriveWithJoystick(bool fieldRelative);

    /**
     * Periodic code for all modes should go here.
     */
    void RobotPeriodic() override;

    /**
     * Initialization code for autonomous mode should go here.
     */
    void AutonomousInit() override;

    /**
     * Initialization code for teleop mode should go here.
     */
    void TeleopInit() override;

    /**
     * Periodic code for autonomous mode should go here.
     */
    void AutonomousPeriodic() override;

    /**
     * Periodic code for teleop mode should go here.
     */
    void TeleopPeriodic() override;
    Robot() = default;
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
