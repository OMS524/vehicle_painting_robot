cmake_minimum_required(VERSION 3.16)
project(doosan_a0912_controller LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(API_DRFL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../API-DRFL")
set(DOOSAN_DRCF_VERSION 2 CACHE STRING "Doosan DRCF major version. Use 2 for M2.x and 3 for M3.x.")

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(DRFL_ARCH "arm64")
else()
  set(DRFL_ARCH "amd64")
endif()

find_program(LSB_RELEASE_EXECUTABLE lsb_release)
if(LSB_RELEASE_EXECUTABLE)
  execute_process(
    COMMAND "${LSB_RELEASE_EXECUTABLE}" -rs
    OUTPUT_VARIABLE DRFL_UBUNTU_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
else()
  set(DRFL_UBUNTU_VERSION "22.04")
  message(WARNING "lsb_release not found; defaulting API-DRFL Ubuntu library version to 22.04")
endif()

if(NOT EXISTS "${API_DRFL_DIR}/library/Linux/64bits/${DRFL_ARCH}/${DRFL_UBUNTU_VERSION}")
  message(FATAL_ERROR "No API-DRFL Linux library directory found for ${DRFL_ARCH}/${DRFL_UBUNTU_VERSION}")
endif()

set(DRFL_LIB_DIR "${API_DRFL_DIR}/library/Linux/64bits/${DRFL_ARCH}/${DRFL_UBUNTU_VERSION}")

add_library(doosan_controller
  src/doosan_controller.cpp
)

target_compile_definitions(doosan_controller PUBLIC
  DRCF_VERSION=${DOOSAN_DRCF_VERSION}
  DOOSAN_HAS_PINOCCHIO=1
)

target_include_directories(doosan_controller PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${API_DRFL_DIR}/include"
)

target_link_libraries(doosan_controller PUBLIC
  "${DRFL_LIB_DIR}/libDRFL.a"
  "${DRFL_LIB_DIR}/libPocoFoundation.so"
  "${DRFL_LIB_DIR}/libPocoNet.so"
  Eigen3::Eigen
  pinocchio::pinocchio
  pthread
)

add_library(doosan_controller_c_api SHARED
  src/doosan_controller_c_api.cpp
)

target_link_libraries(doosan_controller_c_api PRIVATE
  doosan_controller
)

set_target_properties(doosan_controller_c_api PROPERTIES
  BUILD_RPATH "${DRFL_LIB_DIR}"
  INSTALL_RPATH "${DRFL_LIB_DIR}"
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")
  add_executable(doosan_controller_demo
    src/main.cpp
  )

  target_link_libraries(doosan_controller_demo PRIVATE
    doosan_controller
  )

  set_target_properties(doosan_controller_demo PROPERTIES
    BUILD_RPATH "${DRFL_LIB_DIR}"
    INSTALL_RPATH "${DRFL_LIB_DIR}"
  )
endif()

message(STATUS "Using API-DRFL library directory: ${DRFL_LIB_DIR}")
message(STATUS "Using DRCF_VERSION=${DOOSAN_DRCF_VERSION}")
