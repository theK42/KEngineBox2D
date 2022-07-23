cmake_minimum_required(VERSION 3.10)

project(KEngine)

add_library(KEngineBox2D STATIC
Box2DTransform.h
Box2DTransform.cpp
)

set(BOX2D_BUILD_TESTBED OFF)
set(BOX2D_BUILD_UNIT_TESTS OFF)
add_subdirectory(Box2D)
target_link_libraries(KEngineBox2D PRIVATE KEngineCore KEngine2D box2d)
target_include_directories(KEngineBox2D PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})