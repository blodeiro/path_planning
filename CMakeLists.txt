cmake_minimum_required(VERSION 3.0.0)
project(path_planning VERSION 0.1.0)

include_directories(include)

file(GLOB SOURCES "src/*.cpp")

add_executable(path_planning ${SOURCES})