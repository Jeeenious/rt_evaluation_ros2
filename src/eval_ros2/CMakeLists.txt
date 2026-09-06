cmake_minimum_required(VERSION 3.8)
project(eval)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

# 1. 编译为共享库 (Plugin)
add_library(sim_component SHARED src/usr_component.cpp)
ament_target_dependencies(sim_component
        rclcpp
        std_msgs
        rclcpp_components
)

# 2. 注册组件 (这会生成必要的导出信息)
rclcpp_components_register_node(sim_component
        PLUGIN "eval::SimComponent"
        EXECUTABLE sim_node_exec
)

install(TARGETS sim_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

# 3. 自定义多线程容器：线程数由参数 num_threads 决定（供 launch 调用）
add_executable(usr_container src/usr_container.cpp)
ament_target_dependencies(usr_container
        rclcpp
        rclcpp_components
)

install(TARGETS usr_container
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(sim_component)
ament_export_dependencies(rclcpp rclcpp_components std_msgs)

ament_package()