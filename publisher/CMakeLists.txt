cmake_minimum_required(VERSION 3.12)

# Toolchain for working with the NK-parser.
include (platform/nk)

# Set compiler flags.
set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions -Wno-deprecated-declarations")

add_executable (Publisher "src/main.cpp" "src/publisher.cpp" "src/i2c_func.cpp" )
add_dependencies (Publisher publisher_edl_files)
target_link_libraries(Publisher
                      ${mosquitto_CXX_LIB}
                      ${vfs_CLIENT_LIB}
                      ${gpio_CLIENT_LIB}
                      ${bsp_CLIENT_LIB}
                      ${i2c_CLIENT_LIB}
)

# We do not need the default VFS entity which comes from ${vfs_CLIENT_LIB}.
set_target_properties (Publisher PROPERTIES ${vfs_ENTITY}_REPLACEMENT "")
set_target_properties (Publisher PROPERTIES LINK_FLAGS ${VULN_LDFLAGS})
