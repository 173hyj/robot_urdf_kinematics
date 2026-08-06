cmake_minimum_required(VERSION 3.16)

project(robot_urdf_kinematics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED tinyxml2)

add_library(robot_kinematics
    src/Model.cpp
    src/UrdfParser.cpp
    src/KinematicChain.cpp
    src/ForwardKinematics.cpp
)

target_include_directories(robot_kinematics
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${TINYXML2_INCLUDE_DIRS}
)
target_link_directories(robot_kinematics PUBLIC ${TINYXML2_LIBRARY_DIRS})
target_link_libraries(robot_kinematics PUBLIC Eigen3::Eigen ${TINYXML2_LIBRARIES})
target_compile_options(robot_kinematics PRIVATE -Wall -Wextra -Wpedantic)

add_executable(robot_kinematics_cli src/main.cpp)
target_link_libraries(robot_kinematics_cli PRIVATE robot_kinematics)
set_target_properties(robot_kinematics_cli PROPERTIES OUTPUT_NAME robot_kinematics)

add_executable(robot_kinematics_tests tests/test_robot_kinematics.cpp)
target_link_libraries(robot_kinematics_tests PRIVATE robot_kinematics)

enable_testing()
add_test(NAME robot_kinematics_tests COMMAND robot_kinematics_tests)
