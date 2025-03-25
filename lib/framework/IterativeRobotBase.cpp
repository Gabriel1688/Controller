#include "IterativeRobotBase.h"
#include "../ds/DriverStation.h"

//#include <../ds/DriverStation.h>
//#include <networktables/NetworkTableInstance.h>


#include "../ds/DriverStation.h"
#include "../ds/FRCComm.h"
#include "../ds/DSControlWord.h"
#include <iostream>

IterativeRobotBase::IterativeRobotBase(int period)
        : m_period(period) {}

void IterativeRobotBase::RobotInit() {}

void IterativeRobotBase::DriverStationConnected() {}

void IterativeRobotBase::AutonomousInit() {}

void IterativeRobotBase::TeleopInit() {}

void IterativeRobotBase::RobotPeriodic() {
    static bool firstRun = true;
    if (firstRun) {
        std::cout << "Default {}() method... Override me!" << std::endl;
        firstRun = false;
    }
}

void IterativeRobotBase::AutonomousPeriodic() {
    static bool firstRun = true;
    if (firstRun) {
        std::cout << "Default {}() method... Override me!" << std::endl;
        firstRun = false;
    }
}

void IterativeRobotBase::TeleopPeriodic() {
    static bool firstRun = true;
    if (firstRun) {
        std::cout << "Default {}() method... Override me!" << std::endl;
        firstRun = false;
    }
}
void IterativeRobotBase::AutonomousExit() {}

void IterativeRobotBase::TeleopExit() {}


int IterativeRobotBase::GetPeriod() const {
    return m_period;
}

void IterativeRobotBase::LoopFunc() {
    DriverStation::RefreshData();
    // Get current mode
    DSControlWord word;
    Mode mode = Mode::kNone;
    if (word.IsAutonomous()) {
        mode = Mode::kAutonomous;
    } else if (word.IsTeleop()) {
        mode = Mode::kTeleop;
    }

    if (!m_calledDsConnected && word.IsDSAttached()) {
        m_calledDsConnected = true;
        DriverStationConnected();
    }

    // If mode changed, call mode exit and entry functions
    if (m_lastMode != mode) {
        // Call last mode's exit function
        if (m_lastMode == Mode::kAutonomous) {
            AutonomousExit();
        } else if (m_lastMode == Mode::kTeleop) {
            TeleopExit();
        }

        // Call current mode's entry function
        if (mode == Mode::kAutonomous) {
            AutonomousInit();
            //m_watchdog.AddEpoch("AutonomousInit()");
        } else if (mode == Mode::kTeleop) {
            TeleopInit();
            //m_watchdog.AddEpoch("TeleopInit()");
        }

        m_lastMode = mode;
    }

    // Call the appropriate function depending upon the current robot mode
    if (mode == Mode::kAutonomous) {
        //GW HAL_ObserveUserProgramAutonomous();
        AutonomousPeriodic();
        //m_watchdog.AddEpoch("AutonomousPeriodic()");
    } else if (mode == Mode::kTeleop) {
        //GW HAL_ObserveUserProgramTeleop();
        TeleopPeriodic();
        //m_watchdog.AddEpoch("TeleopPeriodic()");
    }

    RobotPeriodic();
}
