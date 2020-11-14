#### e6502 CMAKE FILE
cmake_minimum_required(VERSION 3.1)
project(e6502)

## C++17 COMPILER SUPPORT
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

## PROJECT FILES
include_directories(${CMAKE_SOURCE_DIR}/inc)
add_subdirectory(${CMAKE_SOURCE_DIR}/src)
add_subdirectory(${CMAKE_SOURCE_DIR}/src/component)

## EXECUTABLE
add_executable(${PROJECT_NAME} ${PROJECT_SRC})

## Copy game assets over
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/asset/ $<TARGET_FILE_DIR:${PROJECT_NAME}>/asset/)

target_link_libraries(${PROJECT_NAME} PUBLIC stdc++fs)

## FLAGS
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -g -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG} -pthread -g -Wall -DDEBUG_BUILD")
