cmake_minimum_required (VERSION 3.1)

project(MPC-controller LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "-g -Wall -O3")

# build uWebSockets at configure time locally
if(NOT TARGET uWS)
    set(uWS_EXACT_VERSION 0.13.0)
    configure_file(../downloadUWsSockets.cmake.in uWebSockets-stage/CMakeLists.txt)

    execute_process(
        COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/uWebSockets-stage
    )
    execute_process(
        COMMAND ${CMAKE_COMMAND} --build .
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/uWebSockets-stage
    )

    add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/uWebSockets-src
                     ${CMAKE_CURRENT_BINARY_DIR}/uWebSockets-build)

    file(GLOB uWS_HEADERS ${CMAKE_CURRENT_BINARY_DIR}/uWebSockets-src/src/*.h)
    file(COPY ${uWS_HEADERS} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/uWebSockets-build/include/uWS)
    set(uWS_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR}/uWebSockets-build/include)

    get_target_property(uWS_LIBRARIES uWS INTERFACE_LINK_LIBRARIES)
endif()

add_executable(mpc src/main.cpp src/mpc.cpp)

target_include_directories(mpc PRIVATE ${uWS_INCLUDE_DIRS})
target_link_libraries(mpc PRIVATE ${uWS_LIBRARIES})
