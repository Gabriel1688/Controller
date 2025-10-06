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

message(CHECK_START "Fetching Eigen3")
FetchContent_Declare(
        Eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG master
        GIT_SHALLOW TRUE
        GIT_PROGRESS TRUE)
set(BUILD_TESTING OFF)
set(EIGEN_BUILD_TESTING OFF)
set(EIGEN_MPL2_ONLY ON)
set(EIGEN_BUILD_PKGCONFIG OFF)
set(EIGEN_BUILD_DOC OFF)
FetchContent_MakeAvailable(Eigen)
message(CHECK_PASS "fetched")

#target_link_libraries(YourTarget PRIVATE Eigen3::Eigen)

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
        lib/ds/dsService.cpp

        lib/robot/TimedRobot.cpp
        lib/robot/IterativeRobotBase.cpp
        lib/robot/RobotBase.cpp

        lib/motor/CAN.cpp
        lib/motor/CANAPI.cpp
        lib/motor/TcpClient.cpp
        lib/subsystem/ControlledSubsystemBase.cpp
        lib/subsystem/SubsystemBase.cpp
        lib/subsystem/Drivetrain.cpp
        lib/controllers/DrivetrainController.cpp
        #        lib/motor/CtrlStepMotor.cpp
)

target_include_directories(
        Controller
        PRIVATE
        include
        lib
        ${EIGEN3_INCLUDE_DIR}

)

target_link_libraries(
       Controller
       ssl
       crypto
       websockets
       spdlog
       Eigen3::Eigen
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

        lib/ds/dsService.cpp
        lib/robot/TimedRobot.cpp
        lib/robot/IterativeRobotBase.cpp
        lib/robot/RobotBase.cpp

        lib/motor/CAN.cpp
        lib/motor/CANAPI.cpp
        lib/motor/TcpClient.cpp
        lib/subsystem/Pose2d.h
        lib/controllers/ControllerBase.h
)
target_include_directories(
        framework
        PRIVATE
        include
        lib
        ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(
        framework
        ssl
        crypto
        websockets
        spdlog
        Eigen3::Eigen
)

add_executable(tcpserver src/tcpserver.cpp)
target_compile_options(Controller PRIVATE -Wall -Wextra -Wno-unused-function)