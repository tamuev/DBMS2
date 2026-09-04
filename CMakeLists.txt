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
set(CMAKE_PROJECT_NAME DBMS2)

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

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dbms/blackbox.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dbms/bridge.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dbms/can.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dbms/charging.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dbms/dbms.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dbms/elcon.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dbms/fan.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dbms/faults/faultcheck.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dbms/faults/faults.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dbms/isense.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dbms/j1772.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dbms/ledctl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dbms/model/data.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dbms/model/model.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dbms/precharge.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dbms/sched.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dbms/settings.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dbms/stack.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dbms/storage.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dbms/utils/crc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dbms/utils/eswap.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dbms/utils/lut.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dbms/utils/ma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dbms/vinterface.c
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
