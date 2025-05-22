
#include "RobotBase.h"
#include "mqtt/wrapper.h"
#include "mqtt/mqttClient.h"
#include "motor/CANAPI.h"

namespace hal::init {
    extern void InitializeCANAPI();
}

std::shared_ptr<mqttClient> mqClient;
RobotBase::RobotBase() {
    m_threadId = (unsigned long) pthread_self();

//    SetupMathShared();
#if 0  //TODO:: check connection with driver station.
    auto inst = nt::NetworkTableInstance::GetDefault();
    // subscribe to "" to force persistent values to propagate to local
    nt::SubscribeMultiple(inst.GetHandle(), {{std::string_view{}}});
    if constexpr (!IsSimulation()) {
        inst.StartServer("/home/lvuser/networktables.json");
    } else {
        inst.StartServer();
    }
#endif
    // Call DriverStation::RefreshData() to kick things off
    DriverStation::RefreshData();
}

//TODO:: Initialize the hardware, mqtt and tcp/ip socket.
void InitializeHAL() {
    client_create();
    mqClient = std::shared_ptr<mqttClient>(g_mqttClient_ptr);
    mqClient->loadConfig("../config/config.txt");
//    InitializeCAN();
    hal::init::InitializeCANAPI();
//    InitializeConstants();
//    InitializeCounter();
//    InitializeFRCDriverStation();
//    InitializeMain();
//    InitializeNotifier();
}