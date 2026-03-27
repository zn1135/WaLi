cmake_minimum_required(VERSION 3.22)

enable_language(C  ASM)

set(IMCALIB_Include_Dirs
        ${CMAKE_CURRENT_SOURCE_DIR}/Judge
        ${CMAKE_CURRENT_SOURCE_DIR}/Task
        ${CMAKE_CURRENT_SOURCE_DIR}/Tool
        ${CMAKE_CURRENT_SOURCE_DIR}/User


)

file(GLOB_RECURSE IMCALIB_Application_Src
        ${CMAKE_CURRENT_SOURCE_DIR}/Judge/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Task/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Tool/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/User/*.c
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${IMCALIB_Application_Src}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        ${IMCALIB_Include_Dirs}
)