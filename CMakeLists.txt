# CMake 最低版本号要求
cmake_minimum_required (VERSION 2.8)

# 项目信息
#set (DIR_SRCS ./src)
project (orb_itself)
# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_SRCS 变量
aux_source_directory(./src LIB_SRCS)
aux_source_directory(./testUtil EXE_SRCS)
# 指定生成目标
add_executable(testUtil ${EXE_SRCS})
add_library(ortItself SHARED ${LIB_SRCS})

SET_TARGET_PROPERTIES(ortItself PROPERTIES LINKER_LANGUAGE CXX)
SET_TARGET_PROPERTIES(testUtil PROPERTIES LINKER_LANGUAGE CXX)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
#include_directories( ./inc/TinyVLib )
include_directories( ./include)
# target_link_libraries( stitchImgs ${OpenCV_LIBS} )
target_link_libraries( testUtil ${OpenCV_LIBS} ortItself)
