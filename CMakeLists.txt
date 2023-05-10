# cmake needs this line
# cmake 最低版本需求
cmake_minimum_required(VERSION 3.1)

# Define project name
# 项目名称
project(Demo)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
# 找到Opencv的package
# 这里是wsl所以可以直接寻找因为make install会安装到/usr/local/bin
find_package(OpenCV REQUIRED)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
# message语法可有可无 用于打印输出一些信息
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

# Declare the executable target built from your sources
# 添加可执行文件
add_executable(pro main.cpp)

# Link your application with OpenCV libraries
# 将opencv动态库链接到可执行项目pro
target_link_libraries(pro PRIVATE ${OpenCV_LIBS})
