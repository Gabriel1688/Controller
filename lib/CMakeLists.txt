set(CMAKE_CXX_STANDARD 20)
set (LWS_WITH_CLIENT 1)
set (LWS_ROLE_MQTT 1)
set (LWS_WITH_NETWORK 1)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-local-typedefs  -fPIC -g -O0")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-local-typedefs -Werror -fPIC -g -O0")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_VERBOSE_MAKEFILE ON)

include(FetchContent)
FetchContent_Declare(libwebsockets URL https://github.com/warmcat/libwebsockets/archive/refs/tags/v4.3.5.tar.gz)
FetchContent_MakeAvailable(libwebsockets)

FetchContent_Declare(spdlog URL https://github.com/gabime/spdlog/archive/refs/tags/v1.15.0.tar.gz)
FetchContent_MakeAvailable(spdlog)

message(CHECK_START "Fetching Eigen3")

FetchContent_Declare(Eigen URL https://gitlab.com/libeigen/eigen/-/archive/3.4.1/eigen-3.4.1.tar.gz)
set(BUILD_TESTING OFF)
set(EIGEN_BUILD_TESTING OFF)
set(EIGEN_MPL2_ONLY ON)
set(EIGEN_BUILD_PKGCONFIG OFF)
set(EIGEN_BUILD_DOC OFF)
FetchContent_MakeAvailable(Eigen)
message(CHECK_PASS "fetched")

add_library(framework
        mqtt/wrapper.cpp
        mqtt/mqttClient.cpp

        common/fdevent.cpp
        common/TimerManager.cpp
        common/Synchronization.cpp
        common/Notifier.cpp

        ds/DSControlWord.cpp
        ds/FRCDriverStation.cpp

        ds/BooleanEvent.cpp
        ds/DriverStation.cpp
        ds/GenericHID.cpp
        ds/XboxController.cpp
        ds/EventLoop.cpp
        ds/dsService.cpp

        robot/TimedRobot.cpp
        robot/IterativeRobotBase.cpp
        robot/RobotBase.cpp

        motor/CAN.cpp
        motor/CANAPI.cpp
        motor/TcpClient.cpp
#        motor/CtrlStepMotor.cpp

        subsystem/ControlledSubsystemBase.cpp
        subsystem/SubsystemBase.cpp
        subsystem/Drivetrain.cpp

        controllers/DrivetrainController.cpp
)
set_target_properties(framework PROPERTIES DEBUG_POSTFIX "d")

target_include_directories(framework
        PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/lib
        PUBLIC)

target_link_libraries(
        framework
        PUBLIC
        websockets
        spdlog
        Eigen3::Eigen
)

target_compile_options(framework PRIVATE -Wall -Wextra -Wno-unused-function)
