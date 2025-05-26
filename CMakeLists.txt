cmake_minimum_required(VERSION 3.12.0)
find_program(CMAKE_C_COMPILER NAMES $ENV{CC} gcc PATHS ENV PATH NO_DEFAULT_PATH)
find_program(CMAKE_CXX_COMPILER NAMES $ENV{CXX} g++ PATHS ENV PATH NO_DEFAULT_PATH)

project(Controller)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
 #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-local-typedefs -Werror -fPIC -g -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-local-typedefs  -fPIC -g -O0")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_VERBOSE_MAKEFILE ON)
set (LWS_WITH_CLIENT 1)
set (LWS_ROLE_MQTT 1)
set (LWS_WITH_NETWORK 1)

include(FetchContent)
FetchContent_Declare(libwebsockets URL https://github.com/warmcat/libwebsockets/archive/refs/tags/v4.3.5.tar.gz)
FetchContent_MakeAvailable(libwebsockets)

FetchContent_Declare(spdlog URL https://github.com/gabime/spdlog/archive/refs/tags/v1.15.0.tar.gz)
FetchContent_MakeAvailable(spdlog)

add_executable(
        Controller
        src/main.cpp

        lib/mqtt/wrapper.cpp
        lib/mqtt/mqttClient.cpp

        lib/common/fdevent.cpp
        lib/common/TimerManager.cpp
        lib/common/Synchronization.cpp

        lib/ds/DSControlWord.cpp
        lib/ds/FRCDriverStation.cpp

        lib/ds/BooleanEvent.cpp
        lib/ds/DriverStation.cpp
        lib/ds/GenericHID.cpp
        lib/ds/XboxController.cpp
        lib/ds/EventLoop.cpp

        lib/framework/dsService.cpp
        lib/framework/TimedRobot.cpp
        lib/framework/IterativeRobotBase.cpp
        lib/framework/RobotBase.cpp

        lib/motor/CAN.cpp
        lib/motor/CANAPI.cpp
        lib/motor/TcpClient.cpp
        #        lib/motor/CtrlStepMotor.cpp
)

target_include_directories(
        Controller
        PRIVATE
        include
        lib
)

target_link_libraries(
       Controller
       ssl
       crypto
       websockets
       spdlog
)

add_library(framework

        lib/mqtt/wrapper.cpp
        lib/mqtt/mqttClient.cpp

        lib/common/fdevent.cpp
        lib/common/TimerManager.cpp
        lib/common/Synchronization.cpp

        lib/ds/DSControlWord.cpp
        lib/ds/FRCDriverStation.cpp

        lib/ds/BooleanEvent.cpp
        lib/ds/DriverStation.cpp
        lib/ds/GenericHID.cpp
        lib/ds/XboxController.cpp
        lib/ds/EventLoop.cpp

        lib/framework/dsService.cpp
        lib/framework/TimedRobot.cpp
        lib/framework/IterativeRobotBase.cpp
        lib/framework/RobotBase.cpp

        lib/motor/CAN.cpp
        lib/motor/CANAPI.cpp
        lib/motor/TcpClient.cpp
)
target_include_directories(
        framework
        PRIVATE
        include
        lib
)

target_link_libraries(
        framework
        ssl
        crypto
        websockets
        spdlog
)

add_executable(tcpserver src/tcpserver.cpp)
target_compile_options(Controller PRIVATE -Wall -Wextra -Wno-unused-function)