cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME G474_Example)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        IMCA/Task/ControlTask.c
        IMCA/Task/ControlTask.h
        IMCA/User/user_lib.c
        IMCA/User/user_lib.h
        IMCA/Task/UartTask.c
        IMCA/Task/UartTask.h
        IMCA/Task/CanBus_Task.c
        IMCA/Task/CanBus_Task.h
        IMCA/Tool/Motor.c
        IMCA/Tool/Motor.h
        IMCA/Tool/pid.c
        IMCA/Tool/pid.h
        IMCA/Task/IMU_Task.c
        IMCA/Task/IMU_Task.h
        IMCA/Task/ChassisControl.c
        IMCA/Task/ChassisControl.h
        IMCA/Tool/Ramp.c
        IMCA/Tool/kalman_filter.c
        IMCA/Tool/kalman_filter.h
        IMCA/Tool/INS_task.c
        IMCA/Tool/INS_task.h
        IMCA/Task/GimbalControl.c
        IMCA/Task/GimbalControl.h
        IMCA/Tool/Vision_interact.c
        IMCA/Tool/Vision_interact.h
        IMCA/Task/ShootControl.c
        IMCA/Task/ShootControl.h
        IMCA/Judge/Driver_Judge.c
        IMCA/Judge/Driver_Judge.h
        IMCA/Judge/Judge_interact.c
        IMCA/Judge/Judge_interact.h
        IMCA/Tool/PowerLimit.c
        IMCA/Tool/PowerLimit.h
        IMCA/Tool/CapInteract.c
        IMCA/Tool/CapInteract.h
        IMCA/Judge/UI_STRUCT.h
        IMCA/Judge/ui_picTask.c
        IMCA/Judge/UI.c
        IMCA/Judge/UI.h

)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

add_subdirectory(IMCA)
# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here

)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
#        IMCA/Task
#        IMCA/Tool
#        IMCA/User
#        Drivers/CMSIS/DSP
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
include_directories(
    IMCA/Task
    IMCA/Tool
    IMCA/User
)
