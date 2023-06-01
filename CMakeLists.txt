cmake_minimum_required (VERSION 3.12)
project(mosquitto)

# Initialize CMake library for KasperskyOS SDK.
include (platform)
initialize_platform ()

# Add functions for installing artefacts.
include (platform/install)

# Toolchain for working with the NK-parser.
include (platform/nk)

# Platform definition
include (platform/options)

nk_build_edl_files (publisher_edl_files
                    NK_MODULE "mosquitto" EDL
                    "resources/edl/Publisher.edl")

# Add a package with the mosquitto component.
find_package (mosquitto REQUIRED COMPONENTS CXX_LIB)
include_directories (${mosquitto_INCLUDE})

# Add a package for working with the virtual file system.
find_package (vfs REQUIRED)
include_directories (${vfs_INCLUDE})

# Add a package with the dhcpcd program implementation.
find_package (rump REQUIRED COMPONENTS DHCPCD_ENTITY)
include_directories (${rump_INCLUDE})

# Add a package with the VFS program implementations.
find_package (precompiled_vfs REQUIRED)
include_directories (${precompiled_vfs_INCLUDE})

# Add a package with the GPIO client library.
find_package (gpio REQUIRED COMPONENTS CLIENT_LIB ENTITY)
include_directories (${gpio_INCLUDE})


# This command will find all I2C package components (QEMU_LIB and HW_LIB).
find_package (i2c REQUIRED)
include_directories (${i2c_INCLUDE})

if ("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "arm|aarch64")
    # This command will find the BSP package components
    # (BSP_HW_ENTITY, BSP_QEMU_ENTITY and BSP_CLIENT_LIB)
    find_package (bsp REQUIRED)
    include_directories (${bsp_INCLUDE})
endif ()

# Choose the storage devices for the configuration files.
# If none of the options are selected, the ramfs device is used.
option(ROOTFS_SDCARD "Use SD card to mount root filesystem" OFF)
message(STATUS "Use SD card to mount root filesystem - ${ROOTFS_SDCARD}")

# Set mqtt broker ip address and port.
set (MQTT_BROKER_ADDRESS  192.168.50.29)
set (MQTT_BROKER_PORT     2883)

# Attach the directory with the generated config header files.
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Attach the directories with programs.
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/resources/include)

# Recommended compiler flags against vulnerabilities.
set(VULN_LDFLAGS "\
    -Wl,-z,noexecstack \
    -Wl,-z,separate-code \
    -Wl,-z,now")
set(VULN_COMPILER_FLAGS "\
    -Wall -Wextra -Wconversion \
    -fPIE -pie -D_FORTIFY_SOURCE=2 -O2 \
    -fstack-protector-strong -fstack-clash-protection \
    -mbranch-protection=standard \
    -Wsign-conversion -Wformat=2 -Wformat-security -Werror=format-security \
    -fsanitize=undefined -fsanitize-undefined-trap-on-error")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${VULN_COMPILER_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${VULN_COMPILER_FLAGS}")

add_subdirectory (publisher)

add_subdirectory (einit)
