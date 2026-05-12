cmake_minimum_required(VERSION 3.21)

project(smartfin_ble_client LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SMARTFIN_PROTOCOL_DIR    ${CMAKE_CURRENT_SOURCE_DIR}/src/protocol)
set(SMARTFIN_PROCESSING_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/src/proccessing)
set(SMARTFIN_MATH_DIR        ${CMAKE_CURRENT_SOURCE_DIR}/src/proccessing/math)

option(SMARTFIN_ENABLE_SIMPLEBLE "Enable SimpleBLE transport" ON)

if(SMARTFIN_ENABLE_SIMPLEBLE)
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/SimpleBLE/simpleble/CMakeLists.txt")
        message(FATAL_ERROR
            "SMARTFIN_ENABLE_SIMPLEBLE is ON, but external/SimpleBLE is missing. "
            "Run: git submodule update --init --recursive")
    endif()
    add_subdirectory(external/SimpleBLE/simpleble)
endif()

#  Protocol layer 
# Pure C++20, no platform or BLE dependencies.
# Apple clients (watchOS/iOS) can link this without SimpleBLE.
add_library(smartfin_protocol STATIC
    src/protocol/ensemble_decoder.cpp
)
target_include_directories(smartfin_protocol PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${SMARTFIN_PROTOCOL_DIR}
)
target_compile_features(smartfin_protocol PUBLIC cxx_std_20)

add_library(smartfin_math STATIC
    src/proccessing/math/lin_alg.cpp
)
target_include_directories(smartfin_math PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${SMARTFIN_PROCESSING_DIR}
    ${SMARTFIN_MATH_DIR}
)
target_compile_features(smartfin_math PUBLIC cxx_std_20)

add_library(smartfin_ahrs STATIC
    src/proccessing/AHRS/madgwick_ahrs.cpp
)
target_include_directories(smartfin_ahrs PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/proccessing/AHRS
)
target_link_libraries(smartfin_ahrs PUBLIC
    smartfin_math
    smartfin_protocol
)
target_compile_features(smartfin_ahrs PUBLIC cxx_std_20)

#  Pipeline layer 
# ISampleSink implementations (FileSink, etc). Header-only sinks (BufferSink,
# LoggingSink) need no compilation unit; FileSink does.
add_library(smartfin_pipeline STATIC
    src/pipeline/file_sink.cpp
)
target_include_directories(smartfin_pipeline PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(smartfin_pipeline PUBLIC smartfin_protocol)

#  Processing layer
# Offline processing types: OrientedSample, OrientedRide (Timsort),
# orient_from_imu (AHRS path), orient_from_quat_imu (DMP path).
add_library(smartfin_proc STATIC
    src/proccessing/proc_types.cpp
)
target_include_directories(smartfin_proc PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${SMARTFIN_PROCESSING_DIR}
)
target_link_libraries(smartfin_proc PUBLIC
    smartfin_ahrs
    smartfin_pipeline
)
target_compile_features(smartfin_proc PUBLIC cxx_std_20)

#  Session layer
# Depends on protocol; transport adapter is injected at runtime via IBleAdapter.
add_library(smartfin_session STATIC
    src/session/session.cpp
)
target_include_directories(smartfin_session PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(smartfin_session PUBLIC smartfin_protocol)

#  SimpleBLE transport adapter (optional) 
if(SMARTFIN_ENABLE_SIMPLEBLE)
    add_library(smartfin_transport STATIC
        src/transport/simpleble_adapter.cpp
    )
    target_include_directories(smartfin_transport PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(smartfin_transport PUBLIC
        smartfin_protocol
        simpleble::simpleble
    )
endif()

#  Receiver executable 
add_executable(ble_receiver
    src/receiver/receiver.cpp
)
target_include_directories(ble_receiver PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${SMARTFIN_PROTOCOL_DIR}
    ${SMARTFIN_MATH_DIR}
)
target_link_libraries(ble_receiver PRIVATE
    smartfin_math
    smartfin_protocol
    smartfin_session
)
if(SMARTFIN_ENABLE_SIMPLEBLE)
    target_link_libraries(ble_receiver PRIVATE smartfin_transport)
endif()

#  BLE logger (interactive test tool) 
# Streams ensembles from the fin and prints each one to stdout in real time.
# Usage: ./ble_logger   (Ctrl-C to stop)
if(SMARTFIN_ENABLE_SIMPLEBLE)
    add_executable(ble_logger
        tools/ble_logger.cpp
    )
    target_include_directories(ble_logger PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(ble_logger PRIVATE
        smartfin_protocol
        smartfin_session
        smartfin_transport
    )
endif()

#  Ride recorder (writes binary .sfdat session file) 
if(SMARTFIN_ENABLE_SIMPLEBLE)
    add_executable(ride_recorder
        tools/ride_recorder.cpp
    )
    target_include_directories(ride_recorder PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(ride_recorder PRIVATE
        smartfin_pipeline
        smartfin_protocol
        smartfin_session
        smartfin_transport
    )
endif()
