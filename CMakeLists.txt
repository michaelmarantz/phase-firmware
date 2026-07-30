# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

# Single-source the firmware version from FW_VERSION in main/phase-firmware.c.
# PROJECT_VER becomes the app-descriptor version embedded in the binary,
# which the OTA updater compares against the running version — so bumping
# FW_VERSION is all it takes to make a build "new" to deployed lamps.
file(STRINGS "main/phase-firmware.c" _fw_version_line REGEX "^#define[ \t]+FW_VERSION")
string(REGEX REPLACE ".*\"(.*)\".*" "\\1" PROJECT_VER "${_fw_version_line}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(phase-prototype)
