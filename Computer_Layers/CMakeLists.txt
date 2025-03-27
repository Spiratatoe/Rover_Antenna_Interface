cmake_minimum_required(VERSION 3.24)
project(Computer_Layers)

set(CMAKE_CXX_STANDARD 17)

add_executable(Computer_Layers main.cpp Application_Layer.h Network_Layer.h Data_Link_Layer.h)
