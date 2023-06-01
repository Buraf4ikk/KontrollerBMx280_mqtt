project (einit)

# Tools for working with KasperskyOS images.
include (platform/image)

# Set compiler flags.
project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:NO")

find_program (NTPD_ENTITY
              NAMES "ntpd" REQUIRED
              NO_CMAKE_PATH
              NO_CMAKE_ENVIRONMENT_PATH
              NO_SYSTEM_ENVIRONMENT_PATH)

# Define an ENTITIES variable with a list of programs.
set (ENTITIES
    ${precompiled_vfsVfsNet}
    Publisher
    ${NTPD_ENTITY}
    ${rump_DHCPCD_ENTITY})

if ("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64")
     set (SECURITY_PSL_FILE "src/security.psl")
else ()
     message (FATAL_ERROR "${CMAKE_SYSTEM_PROCESSOR} architecture is not supported." )
endif ()

if (ROOTFS_SDCARD)
    set (ROOTFS_ENTITY "kl.VfsSdCardFs")
    set (FS_BACKEND ${precompiled_vfsVfsSdCardFs})
    set (FS_IMAGE sdcard0.img)
    set (FS_IMG_FORMAT fat32)
    set (FS_IMG_SIZE 64)
    set (QEMU_FLAGS "-drive file=${FS_IMAGE},if=sd,format=raw")
else()
    set (ROOTFS_ENTITY "kl.VfsRamFs")
    set (FS_BACKEND ${precompiled_vfsVfsRamFs})
    set (FS_IMAGE ramdisk0.img)
    set (FS_IMG_FORMAT ext4)
    set (FS_IMG_SIZE 2)
    set (FILES ${CMAKE_BINARY_DIR}/einit/${FS_IMAGE})
endif()

list (APPEND ENTITIES ${FS_BACKEND})

set_target_properties (${FS_BACKEND} PROPERTIES
    EXTRA_ARGS "\
    - -l
    - devfs /dev devfs 0")

set_target_properties (${rump_DHCPCD_ENTITY} PROPERTIES
    DEPENDS_ON_ENTITY ""
    EXTRA_CONNECTIONS "\
  - target: kl.VfsNet
    id: kl.VfsNet
  - target: ${ROOTFS_ENTITY}
    id: ${ROOTFS_ENTITY}"
    EXTRA_ENV "\
    VFS_FILESYSTEM_BACKEND: client{fs->net}:${ROOTFS_ENTITY}
    VFS_NETWORK_BACKEND: client:kl.VfsNet"
    EXTRA_ARGS "\
    - '-4'
    - -f
    - /etc/dhcpcd.conf")

# Add a target to create disk image.
add_custom_target (${FS_IMAGE}
    COMMAND  ${KL_SDK_ROOT_PATH}/common/prepare_hdd_img.sh
        -d   ${CMAKE_SOURCE_DIR}/resources/hdd
        -s   ${FS_IMG_SIZE}
        -f   ${FS_IMG_FORMAT}
        -img ${FS_IMAGE})

# KasperskyOS image for the target hardware platform.
build_kos_hw_image (kos-image
                    EINIT_ENTITY EinitHw
                    CONNECTIONS_CFG "src/init.yaml.in"
                    SECURITY_PSL ${SECURITY_PSL_FILE}
                    IMAGE_FILES  ${ENTITIES}
                                 ${FILES})

if (NOT ROOTFS_SDCARD)
    add_dependencies (kos-image ${FS_IMAGE})
endif ()

# KasperskyOS image for QEMU with simulation targets (sim, gdbsim, gdb).
build_kos_qemu_image (kos-qemu-image
                      EINIT_ENTITY EinitQemu
                      CONNECTIONS_CFG "src/init.yaml.in"
                      SECURITY_PSL ${SECURITY_PSL_FILE}
                      QEMU_FLAGS "${QEMU_FLAGS}"
                      IMAGE_FILES ${ENTITIES}
                                  ${FILES})

add_dependencies (kos-qemu-image ${FS_IMAGE})

set_target_properties (EinitHw PROPERTIES LINK_FLAGS ${VULN_LDFLAGS})
set_target_properties (EinitQemu PROPERTIES LINK_FLAGS ${VULN_LDFLAGS})
