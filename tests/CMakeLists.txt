########################################################################
# Unit tests for TypeHelpers.hpp free functions
########################################################################
add_executable(test_type_helpers test_type_helpers.cpp)

target_include_directories(test_type_helpers PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${SoapySDR_INCLUDE_DIRS}
    ${UHD_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(test_type_helpers PRIVATE
    ${SoapySDR_LIBRARIES}
    ${UHD_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_test(NAME type_helpers COMMAND test_type_helpers)
