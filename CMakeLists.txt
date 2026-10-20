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
set(CMAKE_PROJECT_NAME smartcar)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

set(BASE_SRC 
    ${CMAKE_CURRENT_SOURCE_DIR}/Base/Src/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Base/Src/app_fsm.c
)

# 鎺у埗灞傦細绾畻娉曪紝涓嶄緷璧?HAL
set(CONTROL_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/Control/Src/pid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Control/Src/chassis.c
)

# 鏈嶅姟灞傦細闈㈡澘锛堟寜閿?+ LCD锛変笌閬ユ祴
set(UI_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/UI/Src/panel.c
    ${CMAKE_CURRENT_SOURCE_DIR}/UI/Src/telemetry.c
)

set(CAMARA_PROCESS_SRC 
    ${CMAKE_CURRENT_SOURCE_DIR}/camera_process/Src/Binarization.c
    ${CMAKE_CURRENT_SOURCE_DIR}/camera_process/Src/Element_recognition.c
    ${CMAKE_CURRENT_SOURCE_DIR}/camera_process/Src/scan_line.c
    ${CMAKE_CURRENT_SOURCE_DIR}/camera_process/Src/inner_parameter.c
)

set(HARDWARE_SRC 
    ${CMAKE_CURRENT_SOURCE_DIR}/HardWare/LCD/Src/lcd_fonts.c
    ${CMAKE_CURRENT_SOURCE_DIR}/HardWare/LCD/Src/lcd_image.c
    ${CMAKE_CURRENT_SOURCE_DIR}/HardWare/LCD/Src/lcd_spi_200.c

    ${CMAKE_CURRENT_SOURCE_DIR}/HardWare/OV2640/Src/dcmi_ov2640.c
    ${CMAKE_CURRENT_SOURCE_DIR}/HardWare/OV2640/Src/sccb.c

    ${CMAKE_CURRENT_SOURCE_DIR}/HardWare/motor/Src/motor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/HardWare/motor/Src/encoder.c

    ${CMAKE_CURRENT_SOURCE_DIR}/HardWare/Buzzer/Src/buzzer.c

    ${CMAKE_CURRENT_SOURCE_DIR}/HardWare/protocol/Src/Serial.c
    ${CMAKE_CURRENT_SOURCE_DIR}/HardWare/protocol/Src/bluetooth.c
)


# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CAMARA_PROCESS_SRC}
    ${HARDWARE_SRC}
    ${CONTROL_SRC}
    ${UI_SRC}
    ${BASE_SRC}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Base/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Control/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/UI/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/camera_process/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/HardWare/motor/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/HardWare/LCD/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/HardWare/OV2640/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/HardWare/protocol/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/HardWare/Buzzer/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
