#include "DriverStationModeThread.h"
#include "ds/DriverStation.h"
#include "mqtt/wrapper.h"
#include "mqtt/mqttClient.h"
#include <iostream>
bool m_exit = false;

void StartCompetition() {
    DriverStationModeThread modeThread;

    wpi::Event event{false, false};
    DriverStation::ProvideRefreshedDataEventHandle(event.GetHandle());

    while (!m_exit) {
        if ( DriverStation::IsEnabled()) {
            modeThread.InDisabled(true);
            std::cout << " Robot is Enabled." << std::endl;
            modeThread.InDisabled(false);
            while ( DriverStation::IsEnabled()) {
                wpi::WaitForObject(event.GetHandle());
            }
        }
    }
}



extern "C" {
namespace hal {
    void InitializeDriverStation();
}
    void InitializeFRCDriverStation();
}
std::shared_ptr<mqttClient> mqClient;
int main() {
    client_create();
    mqClient = std::shared_ptr<mqttClient>(g_mqttClient_ptr);
    mqClient->loadConfig("../config/config.txt");
    mqClient->Start();
    InitializeFRCDriverStation();
    hal::InitializeDriverStation();
    DriverStation::RefreshData();
    StartCompetition();
}