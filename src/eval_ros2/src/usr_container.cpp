// 自定义多线程组件容器：线程数由参数 num_threads 决定。
// 生成器(json2launch)把 json 文件名里的 m{x} 解析后作为 num_threads 参数传入，
// 从而让“m”真正等于 executor 线程数（官方 component_container_mt 固定取 CPU 核数）。
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp_components/component_manager.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  // 允许读取 launch 通过 --ros-args -p 传来的参数（含 num_threads）
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  // 预读线程数；缺省 0 → 走 MultiThreadedExecutor 默认值(硬件核数)
  size_t num_threads = 0;
  {
    auto probe = std::make_shared<rclcpp::Node>("component_manager_probe", options);
    if (probe->has_parameter("num_threads")) {
      int v = probe->get_parameter("num_threads").as_int();
      num_threads = (v > 0) ? static_cast<size_t>(v) : 0;
    }
  }

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions(), num_threads);

  auto node = std::make_shared<rclcpp_components::ComponentManager>(
    executor, "ComponentManager", options);

  executor->add_node(node);
  RCLCPP_INFO(node->get_logger(),
    "usr_container started (num_threads=%zu)", num_threads);
  executor->spin();
  rclcpp::shutdown();
  return 0;
}
