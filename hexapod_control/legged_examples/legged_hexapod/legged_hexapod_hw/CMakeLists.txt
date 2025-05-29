cmake_minimum_required(VERSION 3.10)

project(legged_hexapod_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-std=c++17)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    legged_hw
    message_generation
)

add_message_files(
    FILES
    JointCmd.msg
    FootState.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
    INCLUDE_DIRS
    include
    CATKIN_DEPENDS
    roscpp
    legged_hw  
)

###########
## Build ##
###########

# 首先定义路径变量
set(LJL_ROOT_PATH "include/legged_hexapod_hw/ljl")  # 替换为实际路径

# 然后正确引用路径
include_directories(${LJL_ROOT_PATH})
include_directories(${LJL_ROOT_PATH}/Part_Robotrunner/include)
include_directories(${LJL_ROOT_PATH}/Part_Robotrunner/include)
include_directories(${LJL_ROOT_PATH}/Part_HardwareCom/include)
include_directories(${LJL_ROOT_PATH}/Part_Period/include)
include_directories(${LJL_ROOT_PATH}/Eigen)
include_directories(${LJL_ROOT_PATH}/data_struct)

include_directories(${LJL_ROOT_PATH}/include)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

set(SOURCE src/legged_hexapod_hw.cpp src/HexapodHW.cpp)

add_executable(${PROJECT_NAME}
        ${SOURCE}
        )


target_link_libraries(${PROJECT_NAME}   PRIVATE
        ${catkin_LIBRARIES} 
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/libPeriod.so
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/libHardwareCom.so
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/libRobotrunner.so
        ${libevdev_LIBRARIES}
        pthread
        )

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_hexapod_hw
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_hexapod_hw
            CF_WERROR
    )

endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# Mark other files for installation
install(DIRECTORY config launch lib
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )