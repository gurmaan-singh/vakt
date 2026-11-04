cmake_minimum_required(VERSION 3.24)
project(vakt VERSION 0.3.0 DESCRIPTION "Multi-target radar tracker" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(VAKT_BUILD_TESTS "Build unit tests" ON)
option(VAKT_BUILD_APPS "Build command line tools" ON)
option(VAKT_WERROR "Treat warnings as errors" OFF)
option(VAKT_SANITIZE "Build with AddressSanitizer + UBSan (GCC/Clang)" OFF)

include(FetchContent)
set(FETCHCONTENT_QUIET ON)

# Eigen is header only; SOURCE_SUBDIR points nowhere so its own CMake (tests, docs) is skipped.
FetchContent_Declare(eigen
  URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
  URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  SOURCE_SUBDIR do-not-configure)
FetchContent_Declare(json
  URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(eigen json)
add_library(vakt_eigen INTERFACE)
target_include_directories(vakt_eigen SYSTEM INTERFACE ${eigen_SOURCE_DIR})

add_library(vakt_warnings INTERFACE)
# C4324 (padding from alignas) is the point of the cache-line aligned SPSC indices.
if(MSVC AND NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  target_compile_options(vakt_warnings INTERFACE /W4 /permissive- /utf-8 /external:W0 /external:anglebrackets /wd4324
    $<$<BOOL:${VAKT_WERROR}>:/WX>)
else()
  target_compile_options(vakt_warnings INTERFACE -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor
    -Wold-style-cast -Woverloaded-virtual -Wnull-dereference -Wdouble-promotion -Wimplicit-fallthrough
    $<$<BOOL:${VAKT_WERROR}>:-Werror>)
endif()
if(VAKT_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

add_library(vakt
  src/geo.cpp
  src/models.cpp
  src/measurement.cpp
  src/ekf.cpp
  src/imm.cpp
  src/assignment.cpp
  src/tracker.cpp
  src/metrics.cpp
  src/sim.cpp
  src/asterix.cpp
  src/udp.cpp
  src/json_io.cpp
  src/runner.cpp)
add_library(vakt::vakt ALIAS vakt)
target_include_directories(vakt PUBLIC include)
target_link_libraries(vakt PUBLIC vakt_eigen nlohmann_json::nlohmann_json PRIVATE vakt_warnings)
if(WIN32)
  target_link_libraries(vakt PUBLIC ws2_32)
  target_compile_definitions(vakt PUBLIC NOMINMAX WIN32_LEAN_AND_MEAN _USE_MATH_DEFINES)
endif()
find_package(Threads REQUIRED)
target_link_libraries(vakt PUBLIC Threads::Threads)

if(VAKT_BUILD_APPS)
  foreach(app vakt_sim vakt_mc vakt_live vakt_bench)
    add_executable(${app} apps/${app}.cpp)
    target_link_libraries(${app} PRIVATE vakt vakt_warnings)
  endforeach()
endif()

if(VAKT_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
