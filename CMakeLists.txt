cmake_minimum_required(VERSION 3.10)
project(Robot LANGUAGES CXX)

# ==========================================
# C++ 标准 & 构建类型
# ==========================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# ==========================================
# 查找依赖库
# ==========================================

# OpenCV
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV libs: ${OpenCV_LIBS}")

# RealSense2
find_package(realsense2 REQUIRED)

# Python & pybind11
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)

# Add JSON-CPP
find_package(nlohmann_json REQUIRED)

# pthread（Linux）
find_package(Threads REQUIRED)


find_package(CURL REQUIRED)

# ==========================================
# JAKA SDK（Linux / shared / IMPORTED）
# ==========================================
set(JAKA_SDK_ROOT "/opt/jaka_sdk")
set(JAKA_INCLUDE_DIR "${JAKA_SDK_ROOT}/inc_of_c++")
set(JAKA_LIBRARY     "${JAKA_SDK_ROOT}/aarch64-linux-gnu/shared/libjakaAPI.so")

# 检查 JAKA SDK 是否存在
if(NOT EXISTS "${JAKA_INCLUDE_DIR}")
    message(FATAL_ERROR "JAKA include directory not found: ${JAKA_INCLUDE_DIR}")
endif()

if(NOT EXISTS "${JAKA_LIBRARY}")
    message(FATAL_ERROR "JAKA library not found: ${JAKA_LIBRARY}")
endif()

# 定义 IMPORTED library
add_library(JAKA::API SHARED IMPORTED)
set_target_properties(JAKA::API PROPERTIES
    IMPORTED_LOCATION "${JAKA_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${JAKA_INCLUDE_DIR}"
)


# ==========================================
# 源文件
# ==========================================

set(SOURCES
    robot_server.cpp  
    Robot_Flower.cpp
    function.cpp
    robot_core.cpp          # 你封装 InitAll/RunTask/Stop/Log 的文件
)
add_executable(robot_server ${SOURCES})

# ==========================================
# include 目录
# ==========================================

target_include_directories(robot_server PRIVATE
    ${PROJECT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${realsense2_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}
    ${pybind11_INCLUDE_DIRS}
)

# ==========================================
# 链接库
# ==========================================

target_link_libraries(robot_server PRIVATE
    ${OpenCV_LIBS}
    nlohmann_json
    realsense2::realsense2
    CURL::libcurl
    Python3::Python
    JAKA::API
    Threads::Threads
)

# ==========================================
# 编译选项 & 宏
# ==========================================

if(UNIX)
    target_compile_definitions(robot_server PRIVATE __linux__)
    target_compile_options(robot_server PRIVATE -Wall)
endif()

