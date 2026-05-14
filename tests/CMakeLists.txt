add_subdirectory(${CMAKE_SOURCE_DIR}/external/googletest googletest)

add_executable(filtfilt_gust_test
    filtfilt_gust_test.cpp
)

target_include_directories(filtfilt_gust_test PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/proccessing
)

target_link_libraries(filtfilt_gust_test PRIVATE
    smartfin_filter
    smartfin_math
    GTest::gtest_main
)

# Bakes the path to gen_filtfilt_vectors.py into the test binary
target_compile_definitions(filtfilt_gust_test PRIVATE
    TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

include(GoogleTest)
gtest_discover_tests(filtfilt_gust_test)
