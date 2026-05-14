if(EXISTS "${CMAKE_SOURCE_DIR}/external/googletest/CMakeLists.txt")
    add_subdirectory(${CMAKE_SOURCE_DIR}/external/googletest googletest)
else()
    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    )
    FetchContent_MakeAvailable(googletest)
endif()

add_subdirectory(filtfilt)
add_subdirectory(butterworth)
