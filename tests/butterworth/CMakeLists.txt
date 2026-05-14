add_executable(butterworth_test
    butterworth_test.cpp
)

target_include_directories(butterworth_test PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/proccessing
)

target_link_libraries(butterworth_test PRIVATE
    smartfin_filter
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(butterworth_test)
