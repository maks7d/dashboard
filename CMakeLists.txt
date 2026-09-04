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
set(CMAKE_PROJECT_NAME Dash)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# --- Génération automatique de .hex et .bin ---
add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMENT "Génération de ${CMAKE_PROJECT_NAME}.hex et .bin"
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# === FreeRTOS Integration ===
# Path to FreeRTOS kernel
set(FREERTOS_KERNEL_DIR ${CMAKE_SOURCE_DIR}/lib/FreeRTOS-Kernel-11.3.1/)

# FreeRTOS sources - added directly to main target
set(FREERTOS_SOURCES
    ${FREERTOS_KERNEL_DIR}/tasks.c
    ${FREERTOS_KERNEL_DIR}/queue.c
    ${FREERTOS_KERNEL_DIR}/list.c
    ${FREERTOS_KERNEL_DIR}/timers.c
    ${FREERTOS_KERNEL_DIR}/event_groups.c
    ${FREERTOS_KERNEL_DIR}/stream_buffer.c
    ${FREERTOS_KERNEL_DIR}/portable/GCC/ARM_CM7/r0p1/port.c
    ${FREERTOS_KERNEL_DIR}/portable/MemMang/heap_5.c
)

# FreeRTOS includes
set(FREERTOS_INCLUDES
    ${FREERTOS_KERNEL_DIR}/include
    ${FREERTOS_KERNEL_DIR}/portable/GCC/ARM_CM7
    ${FREERTOS_KERNEL_DIR}/portable/GCC/ARM_CM7/r0p1
    ${FREERTOS_KERNEL_DIR}/portable/GCC
    ${FREERTOS_KERNEL_DIR}/portable/Common
    ${FREERTOS_KERNEL_DIR}/portable/MemMang
)

# Add FreeRTOS sources and includes to main target
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${FREERTOS_SOURCES})
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${FREERTOS_INCLUDES})

# Force Thumb mode for all sources (critical for Cortex-M)
set_source_files_properties(${FREERTOS_SOURCES} PROPERTIES COMPILE_FLAGS "-mthumb")
set_source_files_properties(Core/Src/ws2812.c PROPERTIES COMPILE_FLAGS "-mthumb")
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<CONFIG:Debug>:FREERTOS_DEBUG>
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/ws2812.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
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
