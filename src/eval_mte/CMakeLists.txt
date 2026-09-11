cmake_minimum_required(VERSION 3.8)
project(eval_mte)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 查找 ROS2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

# 查找 LTTng-UST
find_package(PkgConfig REQUIRED)
pkg_check_modules(LTTNG_UST REQUIRED lttng-ust)

# 1. 编译组件库
add_library(mte_component SHARED src/mte_component.cpp)

# 包含 LTTng 相关的头文件路径 (关键修正)
target_include_directories(mte_component PRIVATE
    ${LTTNG_UST_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 关键修正 1：防止编译器优化并隐蔽导出符号 (强制预设为 default 导出)
#set_target_properties(mte_component PROPERTIES CXX_VISIBILITY_PRESET default)

ament_target_dependencies(mte_component
    rclcpp
    std_msgs
    rclcpp_components
)

target_link_libraries(mte_component
    ${LTTNG_UST_LIBRARIES}
    ${CMAKE_DL_LIBS}
)

# 2. 注册组件
rclcpp_components_register_node(mte_component
    PLUGIN "eval::MTEComponent"
    EXECUTABLE mte_node_exec
)

install(TARGETS mte_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

# 3. 自定义多线程容器：线程数由参数 num_threads 决定（供 launch 调用）
add_executable(mte_container src/mte_container.cpp)

target_include_directories(mte_container PRIVATE ${LTTNG_UST_INCLUDE_DIRS})

ament_target_dependencies(mte_container
    rclcpp
    rclcpp_components
)

target_link_libraries(mte_container
    ${LTTNG_UST_LIBRARIES}
    ${CMAKE_DL_LIBS}
)

install(TARGETS mte_container
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 4. 导出依赖
ament_export_libraries(mte_component)
ament_export_dependencies(
    rclcpp
    rclcpp_components
    std_msgs
)

ament_package()