cmake_minimum_required(VERSION 3.5)

if(NOT TARGET clang-format)
    include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/compilerSettings.cmake)
endif()

project(ct_hydro VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(ct_hydro INTERFACE)
target_include_directories(ct_hydro INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

set(ct_hydro ct_hydro CACHE INTERNAL "ct_hydro library target.")

include(GNUInstallDirs)

install(DIRECTORY include/ct/hydro DESTINATION include/ct)

install(TARGETS ct_hydro
    EXPORT ct_hydro_export
    INCLUDES DESTINATION include)

install(EXPORT ct_hydro_export DESTINATION share/ct_hydro/cmake)

install(FILES "cmake/ct_hydroConfig.cmake" DESTINATION "share/ct_hydro/cmake")
