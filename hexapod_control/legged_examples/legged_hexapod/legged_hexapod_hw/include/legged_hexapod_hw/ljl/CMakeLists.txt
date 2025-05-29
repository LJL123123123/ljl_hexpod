cmake_minimum_required(VERSION 3.16)
project(code_struct)



set(CMAKE_CXX_STANDARD 20)#设置编译器版本为C++20
set(CMAKE_CXX_STANDARD_REQUIRED ON)# 设置指定的C++编译器版本是必须的
# 设置 C 编译器的优化等级
set(CMAKE_C_FLAGS_RELEASE "-O2") # 为 Release 类型设置 -O2 优化
set(CMAKE_C_FLAGS_DEBUG "-g")    # 为 Debug 类型设置 -g（生成调试信息），不设置优化

# 设置 C++ 编译器的优化等级
set(CMAKE_CXX_FLAGS_RELEASE "-O2") # 为 Release 类型设置 -O2 优化
set(CMAKE_CXX_FLAGS_DEBUG "-g")    # 为 Debug 类型设置 -g（生成调试信息），不设置优化

#分批编译子文件
add_subdirectory(Part_Period)
add_subdirectory(Part_HardwareCom)
add_subdirectory(Part_Robotrunner)

############################################设置主函数############################################
#查找外来库



# 查找 GMP 库
find_library(GMP_LIBRARY gmp)
if(NOT GMP_LIBRARY)
    message(FATAL_ERROR "GMP library not found")
endif()

find_package(PkgConfig REQUIRED)
# pkg_check_modules 函数通过 pkg-config 工具查找名为 libevdev 的模块
# 查找到的结果将被存储在几个变量中，例如：
#     libevdev_INCLUDE_DIRS: 包含库的头文件的目录。
#     libevdev_LIBRARY_DIRS: 包含库的目录。
#     libevdev_LIBRARIES: 库文件的名称。
pkg_check_modules(libevdev REQUIRED libevdev)

# 生成可执行文件
add_executable(code_struct main.cpp Ctr_Hexpod.hpp)

set(LJL_ROOT_PATH ".")  # 替换为实际路径

# 然后正确引用路径
include_directories(${LJL_ROOT_PATH}/Part_Robotrunner/include)
include_directories(${LJL_ROOT_PATH}/Part_Robotrunner/include)
include_directories(${LJL_ROOT_PATH}/Part_HardwareCom/include)
include_directories(${LJL_ROOT_PATH}/Part_Period/include)
include_directories(${LJL_ROOT_PATH}/Eigen)
include_directories(${LJL_ROOT_PATH}/data_struct)

include_directories(${LJL_ROOT_PATH}/include)
# add_executable(code_struct main.cpp Ctr_Hexpod.hpp)

include_directories(${libevdev_INCLUDE_DIRS})
# include_directories(${OpenCV_INCLUDE_DIRS})

# target_include_directories(code_struct PRIVATE ${SymEngine_INCLUDE_DIRS})


# 指定 PUBLIC 会使链接信息和包含路径在目标和依赖目标中都可见。
# 指定 PRIVATE 只会使链接信息在目标中可见，而不会传播给依赖目标。
target_link_libraries(code_struct PRIVATE Period)
target_link_libraries(code_struct PRIVATE HardwareCom)
target_link_libraries(code_struct PRIVATE Robotrunner)
target_link_libraries(code_struct PRIVATE ${libevdev_LIBRARIES})
target_link_libraries(code_struct PRIVATE pthread)


# # 设置头文件搜索路径
# target_include_directories(code_struct PRIVATE ${SymEngine_INCLUDE_DIRS})
# # 链接 SymEngine 库
# target_link_libraries(code_struct PRIVATE /usr/local/lib/libsymengine.a ${GMP_LIBRARY})

# 导入先前编译生成的
target_link_libraries(code_struct PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lib/libRealtimeCurvePlot.so)

set_target_properties(code_struct PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin)