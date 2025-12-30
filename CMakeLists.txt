cmake_minimum_required(VERSION 3.12.0)
find_program(CMAKE_C_COMPILER NAMES $ENV{CC} gcc PATHS ENV PATH NO_DEFAULT_PATH)
find_program(CMAKE_CXX_COMPILER NAMES $ENV{CXX} g++ PATHS ENV PATH NO_DEFAULT_PATH)

project(Controller)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
FetchContent_Declare(spdlog URL https://github.com/gabime/spdlog/archive/refs/tags/v1.15.0.tar.gz)
FetchContent_MakeAvailable(spdlog)

#FetchContent_Declare(fmt
#        GIT_REPOSITORY https://github.com/fmtlib/fmt.git
#        GIT_TAG        10.2.1  # Use a known stable version
#)
add_executable(
        Controller

        src/Robot.cpp
        src/subsystems/Arm.cpp
        src/subsystems/Gripper.cpp
        src/controllers/ArmController.cpp
)

target_include_directories(
        Controller
        PRIVATE
        include
        lib
        src
)

target_link_libraries(
       Controller
       PUBLIC
        framework
)

add_subdirectory(tools)
add_subdirectory(lib)

target_compile_options(Controller PRIVATE -Wall -Wextra -Wno-unused-function)