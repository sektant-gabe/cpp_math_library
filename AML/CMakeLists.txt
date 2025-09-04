cmake_minimum_required(VERSION 3.5)

project(AML)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -std=c++11")

set(SRC_CPP_AML AMLVector3.cpp AMLMAtrix33.cpp)

add_library(${PROJECT_NAME} STATIC ${SRC_CPP_AML})

target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})
