project(common)

set(CMAKE_CXX_STANDARD 20)
set (LWS_WITH_CLIENT 1)
set (LWS_ROLE_MQTT 1)
set (LWS_WITH_NETWORK 1)

include(FetchContent)
FetchContent_Declare(libwebsockets URL https://github.com/warmcat/libwebsockets/archive/refs/tags/v4.3.5.tar.gz)
FetchContent_MakeAvailable(libwebsockets)

add_library(framework
        mqtt/wrapper.cpp
        mqtt/mqttClient.cpp

        common/fdevent.cpp
        common/TimerManager.cpp
        common/Synchronization.cpp
        common/Notifier.cpp

        ds/DSControlWord.cpp
        ds/FRCComm.cpp
        ds/FRCDriverStation.cpp

        ds/BooleanEvent.cpp
        ds/DriverStation.cpp
        ds/GenericHID.cpp
        ds/XboxController.cpp
        ds/EventLoop.cpp

        robot/TimedRobot.cpp
        robot/IterativeRobotBase.cpp
        robot/RobotBase.cpp
        robot/SubsystemBase.cpp

        motor/CAN.cpp
        motor/CANAPI.cpp
        motor/TcpClient.cpp
)
set_target_properties(framework PROPERTIES DEBUG_POSTFIX "d")

target_include_directories(framework
        PRIVATE
        ${CMAKE_SOURCE_DIR}/app/include
        ${CMAKE_SOURCE_DIR}/lib
        PUBLIC)

target_link_libraries(
        framework
        ssl
        crypto
        websockets
        spdlog
)

target_compile_options(framework PRIVATE -Wall -Wextra -Wno-unused-function)