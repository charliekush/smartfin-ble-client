cmake_minimum_required(VERSION 3.21)

project(smartfin_ble_client LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(SMARTFIN_ENABLE_SIMPLEBLE "Enable SimpleBLE transport" ON)

if(SMARTFIN_ENABLE_SIMPLEBLE)
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/SimpleBLE/simpleble/CMakeLists.txt")
        message(FATAL_ERROR
            "SMARTFIN_ENABLE_SIMPLEBLE is ON, but external/SimpleBLE is missing. "
            "Run: git submodule update --init --recursive")
    endif()
    add_subdirectory(external/SimpleBLE/simpleble)
endif()

# ── Protocol layer ────────────────────────────────────────────────────────────
# Pure C++20, no platform or BLE dependencies.
# Apple clients (watchOS/iOS) can link this without SimpleBLE.
add_library(smartfin_protocol STATIC
    src/protocol/ensemble_decoder.cpp
)
target_include_directories(smartfin_protocol PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_compile_features(smartfin_protocol PUBLIC cxx_std_20)

# ── Session layer ─────────────────────────────────────────────────────────────
# Depends on protocol; transport adapter is injected at runtime via IBleAdapter.
add_library(smartfin_session STATIC
    src/session/session.cpp
)
target_include_directories(smartfin_session PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(smartfin_session PUBLIC smartfin_protocol)

# ── SimpleBLE transport adapter (optional) ────────────────────────────────────
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

# ── Receiver executable ───────────────────────────────────────────────────────
add_executable(ble_receiver
    src/receiver/receiver.cpp
)
target_include_directories(ble_receiver PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(ble_receiver PRIVATE
    smartfin_protocol
    smartfin_session
)
if(SMARTFIN_ENABLE_SIMPLEBLE)
    target_link_libraries(ble_receiver PRIVATE smartfin_transport)
endif()
