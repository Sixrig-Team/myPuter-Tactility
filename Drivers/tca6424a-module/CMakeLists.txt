cmake_minimum_required(VERSION 3.20)

include("${CMAKE_CURRENT_LIST_DIR}/../../Buildscripts/module.cmake")

file(GLOB_RECURSE SOURCE_FILES "source/*.c*")

tactility_add_module(tca6424a-module
    SRCS ${SOURCE_FILES}
    INCLUDE_DIRS include/
    REQUIRES TactilityKernel
)
