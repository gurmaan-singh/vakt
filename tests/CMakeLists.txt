FetchContent_Declare(googletest
  URL https://github.com/google/googletest/releases/download/v1.17.0/googletest-1.17.0.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(vakt_tests
  test_models.cpp
  test_filters.cpp
  test_assignment.cpp
  test_metrics.cpp
  test_geo.cpp
  test_asterix.cpp
  test_spsc.cpp
  test_tracker.cpp)
target_link_libraries(vakt_tests PRIVATE vakt vakt_warnings GTest::gtest_main)
target_compile_definitions(vakt_tests PRIVATE VAKT_SCENARIO_DIR="${PROJECT_SOURCE_DIR}/scenarios")

include(GoogleTest)
gtest_discover_tests(vakt_tests DISCOVERY_TIMEOUT 60)
