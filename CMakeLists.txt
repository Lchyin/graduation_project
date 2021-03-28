cmake_minimum_required(VERSION 3.1.0)

project(machine_work)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()
set(CMAKE_CXX_SEANDARD 11)
if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()
# 寻找opencv库
find_package(OpenCV REQUIRED)
FIND_PACKAGE(PkgConfig)    # try find PKGConfig as it will be used if found
find_package(Threads)
# 加入opencv头文件
include_directories(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(./include)
# 添加所需要的输出程序和所 
aux_source_directory(./src dic)
# 添加所需要的输出程序和所需的源文件
add_executable(machine_work ${dic})
target_link_libraries(machine_work  ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})
