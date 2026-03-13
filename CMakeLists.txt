cmake_minimum_required(VERSION 3.16)

project(robot_control_sdk VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMILE_COMMANDS ON)

enable_testing()

add_library(robot_control_sdk
    src/kinematics/fk_solver.cpp
)

target_include_directories(robot_control_sdk
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

add_executable(fk_demo
    examples/fk_demo.cpp
)

target_link_libraries(fk_demo
    PRIVATE
        robot_control_sdk
)

find_package(GTest REQUIRED)

add_executable(test_fk_solver
    tests/test_fk_solver.cpp
)

target_link_libraries(test_fk_solver
    PRIVATE
        robot_control_sdk
        GTest::GTest
        GTest::Main
)

add_test(NAME test_fk_solver COMMAND test_fk_solver)