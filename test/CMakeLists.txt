cmake_minimum_required(VERSION 3.14)
project(SHT2xTests CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
FetchContent_Declare(
  doctest
  GIT_REPOSITORY https://github.com/doctest/doctest.git
  GIT_TAG        v2.4.12
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(doctest)

set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)

# The library under test, built against the fake Arduino API in mocks/ with
# strict warnings and coverage instrumentation.
add_library(sht2x STATIC ${LIBRARY_DIR}/src/SHT2x.cpp)
target_include_directories(sht2x PUBLIC ${LIBRARY_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/mocks)
target_compile_options(sht2x PRIVATE -Wall -Wextra -Werror --coverage -O0 -g)

add_executable(sht2x_tests
  mocks/mocks.cpp
  test_main.cpp
  test_sht2x.cpp
  test_htu2x.cpp
  test_si702x.cpp
)
target_compile_options(sht2x_tests PRIVATE -Wall -Wextra)
target_link_libraries(sht2x_tests PRIVATE sht2x doctest::doctest --coverage)

enable_testing()
add_test(NAME sht2x_tests COMMAND sht2x_tests)
