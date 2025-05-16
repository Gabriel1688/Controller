
#include "RobotBase.h"
#include "../lib/mqtt/wrapper.h"
#include "../lib/mqtt/mqttClient.h"
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
//    InitializeCANAPI();
//    InitializeConstants();
//    InitializeCounter();
//    InitializeFRCDriverStation();
//    InitializeI2C();
//    InitializeInterrupts();
//    InitializeLEDs();
//    InitializeMain();
//    InitializeNotifier();
//    InitializeSPI();
//    InitializeThreads();
}