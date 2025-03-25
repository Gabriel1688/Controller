//
// Created by Gabriel_Wang on 3/13/2025.
//

#include "RobotBase.h"
RobotBase::RobotBase() {
/*
    m_threadId = std::this_thread::get_id();

    SetupCameraServerShared();
    SetupMathShared();

    auto inst = nt::NetworkTableInstance::GetDefault();
    // subscribe to "" to force persistent values to propagate to local
    nt::SubscribeMultiple(inst.GetHandle(), {{std::string_view{}}});
    if constexpr (!IsSimulation()) {
        inst.StartServer("/home/lvuser/networktables.json");
    } else {
        inst.StartServer();
    }

    // wait for the NT server to actually start
    int count = 0;
    while ((inst.GetNetworkMode() & NT_NET_MODE_STARTING) != 0) {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(10ms);
        ++count;
        if (count > 100) {
            wpi::print(stderr, "timed out while waiting for NT server to start\n");
            break;
        }
    }

    connListenerHandle = inst.AddConnectionListener(false, [&](const nt::Event&
    event) {
        if (event.Is(nt::EventFlags::kConnected)) {
            if (event.GetConnectionInfo()->remote_id.starts_with("glass")) {
                HAL_Report(HALUsageReporting::kResourceType_Dashboard,
                           HALUsageReporting::kDashboard_Glass);
                m_dashboardDetected = true;
            } else if (event.GetConnectionInfo()->remote_id.starts_with(
                    "SmartDashboard")) {
                HAL_Report(HALUsageReporting::kResourceType_Dashboard,
                           HALUsageReporting::kDashboard_SmartDashboard);
                m_dashboardDetected = true;
            } else if (event.GetConnectionInfo()->remote_id.starts_with(
                    "shuffleboard")) {
                HAL_Report(HALUsageReporting::kResourceType_Dashboard,
                           HALUsageReporting::kDashboard_Shuffleboard);
                m_dashboardDetected = true;
            } else if (event.GetConnectionInfo()->remote_id.starts_with("elastic") ||
                       event.GetConnectionInfo()->remote_id.starts_with("Elastic")) {
                HAL_Report(HALUsageReporting::kResourceType_Dashboard,
                           HALUsageReporting::kDashboard_Elastic);
                m_dashboardDetected = true;
            } else if (event.GetConnectionInfo()->remote_id.starts_with(
                    "Dashboard")) {
                HAL_Report(HALUsageReporting::kResourceType_Dashboard,
                           HALUsageReporting::kDashboard_LabVIEW);
                m_dashboardDetected = true;
            } else if (event.GetConnectionInfo()->remote_id.starts_with(
                    "AdvantageScope")) {
                HAL_Report(HALUsageReporting::kResourceType_Dashboard,
                           HALUsageReporting::kDashboard_AdvantageScope);
                m_dashboardDetected = true;
            } else if (event.GetConnectionInfo()->remote_id.starts_with(
                    "QFRCDashboard")) {
                HAL_Report(HALUsageReporting::kResourceType_Dashboard,
                           HALUsageReporting::kDashboard_QFRCDashboard);
                m_dashboardDetected = true;
            } else if (event.GetConnectionInfo()->remote_id.starts_with(
                    "FRC Web Components")) {
                HAL_Report(HALUsageReporting::kResourceType_Dashboard,
                           HALUsageReporting::kDashboard_FRCWebComponents);
                m_dashboardDetected = true;
            } else {
                if (!m_dashboardDetected) {
                    size_t delim = event.GetConnectionInfo()->remote_id.find('@');
                    if (delim != std::string::npos) {
                        HAL_Report(
                                HALUsageReporting::kResourceType_Dashboard,
                                HALUsageReporting::kDashboard_Unknown, 0,
                                event.GetConnectionInfo()->remote_id.substr(0, delim).c_str());
                    } else {
                        HAL_Report(HALUsageReporting::kResourceType_Dashboard,
                                   HALUsageReporting::kDashboard_Unknown, 0,
                                   event.GetConnectionInfo()->remote_id.c_str());
                    }
                }
            }
        }
    });

    SmartDashboard::init();

    if constexpr (!IsSimulation()) {
        std::FILE* file = nullptr;
        file = std::fopen("/tmp/frc_versions/FRC_Lib_Version.ini", "w");

        if (file != nullptr) {
            std::fputs("C++ ", file);
            std::fputs(GetWPILibVersion(), file);
            std::fclose(file);
        }
    }

    // Call DriverStation::RefreshData() to kick things off
    DriverStation::RefreshData();

    // First and one-time initialization
    LiveWindow::SetEnabled(false);
*/
}
