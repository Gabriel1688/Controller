#include "SubsystemBase.h"

std::vector<SubsystemBase *> SubsystemBase::m_subsystems;

SubsystemBase::SubsystemBase() { m_subsystems.emplace_back(this); }

SubsystemBase::~SubsystemBase() {
    m_subsystems.erase(std::remove(m_subsystems.begin(), m_subsystems.end(), this));
}

void SubsystemBase::RunAllDisabledInit() {
    for (auto &subsystem : m_subsystems) {
        subsystem->DisabledInit();
    }
}

void SubsystemBase::RunAllAutonomousInit() {
    for (auto &subsystem : m_subsystems) {
        subsystem->AutonomousInit();
    }
}

void SubsystemBase::RunAllTeleopInit() {
    ConsumeButtonEdgeEvents();

    for (auto &subsystem : m_subsystems) {
        subsystem->TeleopInit();
    }
}

void SubsystemBase::RunAllRobotPeriodic() {
    for (auto &subsystem : m_subsystems) {
        subsystem->RobotPeriodic();
    }
}

void SubsystemBase::RunAllDisabledPeriodic() {
    for (auto &subsystem : m_subsystems) {
        subsystem->DisabledPeriodic();
    }
}

void SubsystemBase::RunAllAutonomousPeriodic() {
    for (auto &subsystem : m_subsystems) {
        subsystem->AutonomousPeriodic();
    }
}

void SubsystemBase::RunAllTeleopPeriodic() {
    for (auto &subsystem : m_subsystems) {
        subsystem->TeleopPeriodic();
    }
}

void SubsystemBase::ConsumeButtonEdgeEvents() {
    // Consumes button edge events produced in disabled mode
    //    for (int stick = 0; stick < frc::DriverStation::kJoystickPorts; ++stick) {
    //        for (int button = 1; button < 32; ++button) {
    //            DriverStation::GetStickButtonPressed(stick, button);
    //            DriverStation::GetStickButtonReleased(stick, button);
    //        }
    //    }
}
