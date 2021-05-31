cmake_minimum_required(VERSION 3.13...3.18)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(MAIN_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(MAIN_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib")
set(MAIN_UTILS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/utils")
set(MAIN_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/main")
set(SVD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dev/svd")
set(DOWNLOADS_DIR "${MAIN_DIR}/downloads")
set(TOOLS_DIR "${MAIN_DIR}/tools")

set(TOOLCHAIN_OPTIONS none arm-none-eabi)
set(TOOLCHAIN "arm-none-eabi" CACHE STRING "Toolchain to use. Available: ${TOOLCHAIN_OPTIONS}")
set_property(CACHE TOOLCHAIN PROPERTY STRINGS ${TOOLCHAIN_OPTIONS})
if("" STREQUAL TOOLCHAIN)
    set(TOOLCHAIN none)
endif()
if (NOT ${TOOLCHAIN} IN_LIST TOOLCHAIN_OPTIONS)
    message(FATAL_ERROR "Invalid toolchain ${TOOLCHAIN}. Valid options are: ${TOOLCHAIN_OPTIONS}")
endif()

option(COMPILER_VERSION_CHECK "Ensure the compiler matches the expected version" ON)

include(GetGitRevisionDescription)
get_git_head_revision(GIT_REFSPEC GIT_SHA1)
string(SUBSTRING ${GIT_SHA1} 0 8 GIT_REV)

# Load settings related functions, so the tests can use them
include(main)
include(settings)

if(TOOLCHAIN STREQUAL none)
    add_subdirectory(src/test)
else()
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${TOOLCHAIN}.cmake")
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/${TOOLCHAIN}-checks.cmake")
endif()

project(INAV VERSION 3.1.0)

enable_language(ASM)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_program(RUBY_EXECUTABLE ruby)
if (NOT RUBY_EXECUTABLE)
    message(FATAL_ERROR "Could not find ruby")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    set(IS_RELEASE_BUILD ON)
endif()

set(FIRMWARE_VERSION ${PROJECT_VERSION})

option(WARNINGS_AS_ERRORS "Make all warnings into errors")
message("-- toolchain: ${TOOLCHAIN}, WARNINGS_AS_ERRORS: ${WARNINGS_AS_ERRORS}")

set(COMMON_COMPILE_DEFINITIONS
    FC_VERSION_MAJOR=${CMAKE_PROJECT_VERSION_MAJOR}
    FC_VERSION_MINOR=${CMAKE_PROJECT_VERSION_MINOR}
    FC_VERSION_PATCH_LEVEL=${CMAKE_PROJECT_VERSION_PATCH}
)

include(openocd)
include(svd)
include(stm32)

add_subdirectory(src)

collect_targets()

message("-- Build type: ${CMAKE_BUILD_TYPE}")
include(ci)
