﻿CMAKE_MINIMUM_REQUIRED(VERSION 3.5)

PROJECT(CIDP-RM-CV)
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_BUILD_TYPE RELEASE)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPATH=\"\\\"${PROJECT_SOURCE_DIR}\\\"\"")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D${CMAKE_SYSTEM_NAME}")
SET(YAML_CPP_LIBRARIES /usr/local/lib/libyaml-cpp.so)  # 设置yaml-cpp库
SET(HIKVISION_CAMERA_DIR "${PROJECT_SOURCE_DIR}/camera/hikvision") # 设置海康相机SDK工具包目录
SET(BIN_NAME "run")


FIND_FILE(CONFIG_FOUND "config.h" "others/include/config")
if (CONFIG_FOUND)
    MESSAGE("Found config.h")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_CONFIG")
endif()

add_compile_definitions(PROJECT_DIR="${PROJECT_SOURCE_DIR}")

#FIND_PROGRAM(CCACHE_FOUND ccache) # ccache编译器缓存
#IF(CCACHE_FOUND)
#    SET_PROPERTY(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
#    SET_PROPERTY(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
#    MESSAGE("< Use ccache for compiler >")
#ENDIF()

FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(OpenCV 4 REQUIRED) # 数字根据实际使用opencv版本更改,例如opencv3.X就改为3，opencv4.X就改为4
FIND_PACKAGE(Threads)

# 导入海康相机SDK
LINK_DIRECTORIES(${HIKVISION_CAMERA_DIR}/64)
INCLUDE_DIRECTORIES(${HIKVISION_CAMERA_DIR}/include)
INCLUDE_DIRECTORIES(${HIKVISION_CAMERA_DIR}/tool/include)

LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/others)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/energy/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/armor/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/others/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/angle)


FILE(GLOB_RECURSE sourcefiles "others/src/*.cpp" "energy/src/*cpp" "armor/src/*.cpp"  "angle/*.cpp")
FILE(GLOB_RECURSE Hiksourcefiles "${HIKVISION_CAMERA_DIR}/src/*.cpp" "${HIKVISION_CAMERA_DIR}/tool/src/*.cpp") # 海康相机拓展工具文件
ADD_EXECUTABLE(${BIN_NAME} main.cpp ${sourcefiles} ${Hiksourcefiles})

TARGET_LINK_LIBRARIES(${BIN_NAME} ${CMAKE_THREAD_LIBS_INIT})
TARGET_LINK_LIBRARIES(${BIN_NAME} ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(${BIN_NAME} MvCameraControl)
TARGET_LINK_LIBRARIES(${BIN_NAME} ${YAML_CPP_LIBRARIES})


# 在不同系统导入迈德威视库的方法（以实际项目相机动态连接库所在路径为准！）
IF (CMAKE_SYSTEM_NAME MATCHES "Linux")          # Linux系统
    MESSAGE(STATUS "current platform: Linux ")
    TARGET_LINK_LIBRARIES(${BIN_NAME} "${PROJECT_SOURCE_DIR}/others/libMVSDK.so")
ELSEIF (CMAKE_SYSTEM_NAME MATCHES "Windows")    # Windows系统
    MESSAGE(STATUS "current platform: Windows")
    TARGET_LINK_LIBRARIES(${BIN_NAME} "${PROJECT_SOURCE_DIR}/others/MVCAMSDK_X64.lib")
ELSEIF (CMAKE_SYSTEM_NAME MATCHES "Darwin")     # Mac系统
    MESSAGE(STATUS "current platform: Mac")
    TARGET_LINK_LIBRARIES(${BIN_NAME} "${PROJECT_SOURCE_DIR}/others/libmvsdk.dylib")
ELSE ()
    MESSAGE(STATUS "Unsupport platform: ${CMAKE_SYSTEM_NAME}")
ENDIF()

# 设置海康相机配置目录路径
set(HIK_CONFIG_FILE_PATH ${PROJECT_SOURCE_DIR}/camera/hikvision/config)
target_compile_definitions(${BIN_NAME} PRIVATE "HIK_CONFIG_FILE_PATH=\"${HIK_CONFIG_FILE_PATH}\"")

# 设置相机标定相关的配置文件路径
set(HIK_CALI_FILE_PATH ${PROJECT_SOURCE_DIR}/camera/hikvision/config/cameraCaliData)
target_compile_definitions(${BIN_NAME} PRIVATE "HIK_CALI_FILE_PATH=\"${HIK_CALI_FILE_PATH}\"")

ADD_CUSTOM_TARGET(create-startup COMMAND "${PROJECT_SOURCE_DIR}/tools/create-startup.sh" "${PROJECT_SOURCE_DIR}" "${CMAKE_BINARY_DIR}")
ADD_CUSTOM_TARGET(train-cnn COMMAND "gnome-terminal" "--" "bash" "-c" "${PROJECT_SOURCE_DIR}/tools/TrainCNN/backward.py")
