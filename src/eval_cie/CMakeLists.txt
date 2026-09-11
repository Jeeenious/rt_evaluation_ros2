cmake_minimum_required(VERSION 3.8)
project(eval_cie)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 查找 ROS2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(callback_isolated_executor REQUIRED)  # CIE 特有依赖

# 查找 LTTng-UST
find_package(PkgConfig REQUIRED)
pkg_check_modules(LTTNG_UST REQUIRED lttng-ust)

# 1. 编译组件库
add_library(cie_component SHARED src/cie_component.cpp)

# 包含 LTTng 相关的头文件路径 (关键修正)
target_include_directories(cie_component PRIVATE
    ${LTTNG_UST_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 关键修正 1：防止编译器优化并隐蔽导出符号 (强制预设为 default 导出)
#set_target_properties(cie_component PROPERTIES CXX_VISIBILITY_PRESET default)

ament_target_dependencies(cie_component
    rclcpp
    std_msgs
    rclcpp_components
    callback_isolated_executor  # 添加 CIE 依赖
)

target_link_libraries(cie_component
    ${LTTNG_UST_LIBRARIES}
    ${CMAKE_DL_LIBS}
)

# 注册组件 (生成可执行文件入口)
rclcpp_components_register_node(cie_component
        PLUGIN "eval::CIEComponent"
        EXECUTABLE cie_node_exec
)

# 安装库文件
install(TARGETS cie_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# 3. 自定义多线程容器：线程数由参数 num_threads 决定（供 launch 调用）
add_executable(cie_container src/cie_container.cpp)

ament_target_dependencies(cie_container
        rclcpp
        rclcpp_components
        callback_isolated_executor  # CIE 容器需要
)

target_link_libraries(cie_container
    ${LTTNG_UST_LIBRARIES}
    ${CMAKE_DL_LIBS}
)

# 安装容器可执行文件
install(TARGETS cie_container
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 3. 导出信息
ament_export_libraries(cie_component)
ament_export_dependencies(
        rclcpp
        rclcpp_components
        std_msgs
        callback_isolated_executor
)

ament_package()