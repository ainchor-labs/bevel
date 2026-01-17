cmake_minimum_required(VERSION 3.18)
project(bevel_jolt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find Python and pybind11
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG)

# If pybind11 not found via config, try to find it via Python
if(NOT pybind11_FOUND)
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -c "import pybind11; print(pybind11.get_cmake_dir())"
        OUTPUT_VARIABLE pybind11_DIR
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    find_package(pybind11 CONFIG REQUIRED)
endif()

# Jolt Physics via FetchContent
include(FetchContent)
FetchContent_Declare(
    JoltPhysics
    GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics
    GIT_TAG v5.0.0
    GIT_SHALLOW TRUE
)

# Configure Jolt options
set(DOUBLE_PRECISION OFF CACHE BOOL "" FORCE)
set(GENERATE_DEBUG_SYMBOLS ON CACHE BOOL "" FORCE)
set(CROSS_PLATFORM_DETERMINISTIC OFF CACHE BOOL "" FORCE)
set(INTERPROCEDURAL_OPTIMIZATION ON CACHE BOOL "" FORCE)
set(FLOATING_POINT_EXCEPTIONS_ENABLED OFF CACHE BOOL "" FORCE)
set(OBJECT_LAYER_BITS 16 CACHE STRING "" FORCE)
set(USE_SSE4_1 ON CACHE BOOL "" FORCE)
set(USE_SSE4_2 ON CACHE BOOL "" FORCE)
set(USE_AVX OFF CACHE BOOL "" FORCE)
set(USE_AVX2 OFF CACHE BOOL "" FORCE)
set(USE_AVX512 OFF CACHE BOOL "" FORCE)
set(USE_LZCNT OFF CACHE BOOL "" FORCE)
set(USE_TZCNT OFF CACHE BOOL "" FORCE)
set(USE_F16C OFF CACHE BOOL "" FORCE)
set(USE_FMADD OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(DOUBLE_PRECISION OFF CACHE BOOL "" FORCE)
set(GENERATE_DEBUG_SYMBOLS ON CACHE BOOL "" FORCE)
set(CROSS_PLATFORM_DETERMINISTIC OFF CACHE BOOL "" FORCE)
set(INTERPROCEDURAL_OPTIMIZATION ON CACHE BOOL "" FORCE)
set(FLOATING_POINT_EXCEPTIONS_ENABLED OFF CACHE BOOL "" FORCE)
set(OBJECT_LAYER_BITS 16 CACHE STRING "" FORCE)
set(USE_SSE4_1 ON CACHE BOOL "" FORCE)
set(USE_SSE4_2 ON CACHE BOOL "" FORCE)
set(USE_AVX OFF CACHE BOOL "" FORCE)
set(USE_AVX2 OFF CACHE BOOL "" FORCE)
set(USE_AVX512 OFF CACHE BOOL "" FORCE)
set(USE_LZCNT OFF CACHE BOOL "" FORCE)
set(USE_TZCNT OFF CACHE BOOL "" FORCE)
set(USE_F16C OFF CACHE BOOL "" FORCE)
set(USE_FMADD OFF CACHE BOOL "" FORCE)

# Try to fetch Jolt Physics
set(JOLT_AVAILABLE FALSE)
FetchContent_GetProperties(JoltPhysics)

if(NOT JoltPhysics_POPULATED)
    message(STATUS "Fetching Jolt Physics...")
    if(FetchContent_Populate(JoltPhysics))
        if(EXISTS "${JoltPhysics_SOURCE_DIR}/CMakeLists.txt")
            add_subdirectory(${JoltPhysics_SOURCE_DIR} ${JoltPhysics_BINARY_DIR} EXCLUDE_FROM_ALL)
            if(TARGET Jolt)
                set(JOLT_AVAILABLE TRUE)
            endif()
        endif()
    endif()
endif()

if(NOT JOLT_AVAILABLE)
    message(WARNING "Jolt Physics not available - Jolt bindings will be skipped")
    message(WARNING "You can install Jolt Physics from: https://github.com/jrouwe/JoltPhysics")
    # Create an empty target to satisfy setuptools
    add_library(_jolt SHARED jolt_wrapper.cpp)
    target_link_libraries(_jolt PRIVATE pybind11::module)
    set_target_properties(_jolt PROPERTIES
        PREFIX ""
        SUFFIX ".so"
    )
    return()
endif()

# Build the Python module with Jolt
pybind11_add_module(_jolt
    jolt_wrapper.cpp
)

target_include_directories(_jolt PRIVATE
    ${JoltPhysics_SOURCE_DIR}/..
)

target_link_libraries(_jolt PRIVATE Jolt)

# Set output name
set_target_properties(_jolt PROPERTIES
    OUTPUT_NAME "_jolt"
    PREFIX ""
)

# Install to the bevel package
install(TARGETS _jolt
    LIBRARY DESTINATION bevel/physics/jolt_bindings
    RUNTIME DESTINATION bevel/physics/jolt_bindings
)
