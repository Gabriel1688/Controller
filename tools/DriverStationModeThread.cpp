#include <iostream>
#include <ds/DriverStation.h>
#include <common/Synchronization.h>
#include "DriverStationModeThread.h"

DriverStationModeThread::DriverStationModeThread() {
    m_keepAlive = true;
    m_thread = std::thread{[&] { Run(); }};
}

DriverStationModeThread::~DriverStationModeThread() {
    m_keepAlive = false;
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

void DriverStationModeThread::InDisabled(bool entering) {
    m_userInDisabled = entering;
}

void DriverStationModeThread::InAutonomous(bool entering) {
    m_userInAutonomous = entering;
}

void DriverStationModeThread::InTeleop(bool entering) {
    m_userInTeleop = entering;
}

void DriverStationModeThread::InTest(bool entering) {
    m_userInTest = entering;
}

void DriverStationModeThread::Run() {
    wpi::Event event{false, false};
    HAL_ProvideNewDataEventHandle(event.GetHandle());

    while (m_keepAlive.load()) {
        bool timedOut = false;
        wpi::WaitForObject(event.GetHandle());

        //wpi::WaitForObject(event.GetHandle(), 0.1, &timedOut);

        DriverStation::RefreshData();

        if (m_userInDisabled) {
           std::cout <<"DriverStationModeThread::Run user disabled." << std::endl;
        }
        if (m_userInAutonomous) {
            std::cout <<"DriverStationModeThread::Run InAutonomous." << std::endl;
        }
        if (m_userInTeleop) {
            std::cout <<"DriverStationModeThread::Run InTeleop." << std::endl;
        }
    }
    HAL_RemoveNewDataEventHandle(event.GetHandle());
}
