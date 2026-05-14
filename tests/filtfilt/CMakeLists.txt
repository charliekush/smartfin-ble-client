find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(GEN_SCRIPT  ${CMAKE_CURRENT_SOURCE_DIR}/gen_filtfilt_vectors.py)
set(GEN_HEADER  ${CMAKE_CURRENT_BINARY_DIR}/filtfilt_test_vectors.hpp)

add_custom_command(
    OUTPUT  ${GEN_HEADER}
    COMMAND ${Python3_EXECUTABLE} ${GEN_SCRIPT} ${GEN_HEADER}
    DEPENDS ${GEN_SCRIPT}
    COMMENT "Generating filtfilt test vectors"
    VERBATIM
)

add_custom_target(filtfilt_vectors_header DEPENDS ${GEN_HEADER})

add_executable(filtfilt_gust_test
    filtfilt_gust_test.cpp
)

add_executable(filtfilt_properties_test
    filtfilt_properties_test.cpp
)

add_dependencies(filtfilt_gust_test filtfilt_vectors_header)

target_include_directories(filtfilt_gust_test PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/proccessing
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(filtfilt_gust_test PRIVATE
    smartfin_filter
    smartfin_math
    GTest::gtest_main
)

target_include_directories(filtfilt_properties_test PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/proccessing
)

target_link_libraries(filtfilt_properties_test PRIVATE
    smartfin_filter
    smartfin_math
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(filtfilt_gust_test)
gtest_discover_tests(filtfilt_properties_test)
