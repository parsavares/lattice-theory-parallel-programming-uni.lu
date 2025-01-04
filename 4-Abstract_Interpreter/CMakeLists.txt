cmake_minimum_required(VERSION 3.24)
cmake_policy(SET CMP0135 NEW)

project(absint
    HOMEPAGE_URL "https://github.com/ptal/lattice-theory-parallel-programming-uni.lu"
    LANGUAGES CXX)

# ------------------------------------------------
# Fetch cpp-peglib (for parsing C programs)
# ------------------------------------------------
include(FetchContent)
FetchContent_Declare(
    cpp_peglib
    GIT_REPOSITORY "https://github.com/ptal/cpp-peglib.git"
    GIT_TAG        245b59446d424b499b316ee77ec59de18443de23
)
FetchContent_MakeAvailable(cpp_peglib)

add_executable(absint
    src/main.cpp
    src/abstract_interpreter.cpp
)
target_include_directories(absint PRIVATE include)
target_compile_features(absint PRIVATE cxx_std_17)
target_link_libraries(absint PRIVATE cpp_peglib)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(absint PRIVATE DEBUG)
    message("Debug mode enabled")
endif()