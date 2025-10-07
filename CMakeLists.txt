cmake_minimum_required(VERSION 3.12.0)
find_program(CMAKE_C_COMPILER NAMES $ENV{CC} gcc PATHS ENV PATH NO_DEFAULT_PATH)
find_program(CMAKE_CXX_COMPILER NAMES $ENV{CXX} g++ PATHS ENV PATH NO_DEFAULT_PATH)

project(Controller)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(
        Controller
        src/main.cpp
)

target_include_directories(
        Controller
        PRIVATE
        include
        lib
)

target_link_libraries(
       Controller
       PRIVATE
        framework
)

add_subdirectory(tools)
add_subdirectory(lib)

target_compile_options(Controller PRIVATE -Wall -Wextra -Wno-unused-function)