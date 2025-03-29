cmake_minimum_required(VERSION 3.15)
project(UDPListener)

set(CMAKE_CXX_STANDARD 14)

add_executable(UDPListener main.cpp)

if(WIN32)
    target_link_libraries(UDPListener PRIVATE ws2_32)
endif()
