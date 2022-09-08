cmake_minimum_required(VERSION 3.8)
project(ui_process_manager)

# Set compile options
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
endif ()
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif ()

# Ignore PCL errors in Clang
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wno-gnu-anonymous-struct -Wno-nested-anon-types)
endif ()


# Set ROS_DISTRO macros
set(ROS_DISTRO $ENV{ROS_DISTRO})
if (${ROS_DISTRO} STREQUAL "rolling")
    add_compile_definitions(ROS_DISTRO_ROLLING)
elseif (${ROS_DISTRO} STREQUAL "galactic")
    add_compile_definitions(ROS_DISTRO_GALACTIC)
elseif (${ROS_DISTRO} STREQUAL "humble")
    add_compile_definitions(ROS_DISTRO_HUMBLE)
endif ()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Boost REQUIRED COMPONENTS system filesystem)

ament_auto_add_executable(ui_process_manager_node
        src/main.cpp
        src/ui_process_manager.cpp
        )

target_include_directories(ui_process_manager_node
        SYSTEM PUBLIC
        ${BOOST_INCLUDE_DIRS}
        )

ament_target_dependencies(ui_process_manager_node
        Boost
        )

ament_auto_package(
        INSTALL_TO_SHARE
        launch
        config
)
