cmake_minimum_required(VERSION 3.20)
project(motor_can_cpp)

#############################################本地库生成#############################################
# Set the include directoriesW

# find dependencies
include_directories(include)  # 确保包含了 include 目录
include_directories(../Part_Period/include)
include_directories(../data_struct)
include_directories(./include/motor_can_cpp)
include_directories(../Part_HardwareCom/include)


set(SOURCES src/motor_can.cpp)
#Generate the shared library把该生成的项目创建为共享库（创建的内容为上面定义的SOURCES内容）
add_library(motor_can_cpp SHARED ${SOURCES})

target_link_libraries(motor_can_cpp PRIVATE Period)

#指定了ThreadPool这个对象(上一步创建的shared库)，并将其地址设置为build/lib
set_target_properties(motor_can_cpp PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../lib)
#############################################可执行文件输出#############################################


