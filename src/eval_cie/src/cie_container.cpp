#include <memory>
#include <string>
#include <vector>
#include <sched.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"
#include "callback_isolated_executor/callback_isolated_executor.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  // --------------------------------------------------
  // 1. 从参数读取配置
  // --------------------------------------------------
  std::size_t num_threads = 0;
  int thread_priority = 50;
  std::string scheduler_policy = "SCHED_FIFO";
  std::vector<int> cpu_affinity = {0, 1, 2};

  {
    auto probe = std::make_shared<rclcpp::Node>("cie_probe", options);

    if (probe->has_parameter("num_threads")) {
      int v = probe->get_parameter("num_threads").as_int();
      if (v > 0) num_threads = static_cast<std::size_t>(v);
    }

    if (probe->has_parameter("thread_priority")) {
      int v = probe->get_parameter("thread_priority").as_int();
      if (v > 0 && v <= 99) thread_priority = v;
    }

    if (probe->has_parameter("scheduler_policy")) {
      std::string v = probe->get_parameter("scheduler_policy").as_string();
      if (v == "SCHED_FIFO" || v == "SCHED_RR" || v == "SCHED_OTHER") {
        scheduler_policy = v;
      }
    }

    if (probe->has_parameter("cpu_affinity")) {
      auto v = probe->get_parameter("cpu_affinity").as_integer_array();
      if (!v.empty()) {
        cpu_affinity.clear();
        for (const auto& core : v) {
          cpu_affinity.push_back(static_cast<int>(core));
        }
      }
    }
  }

  // --------------------------------------------------
  // 2. 转换为调度策略宏
  // --------------------------------------------------
  int scheduler = SCHED_FIFO;
  if (scheduler_policy == "SCHED_FIFO") {
    scheduler = SCHED_FIFO;
  } else if (scheduler_policy == "SCHED_RR") {
    scheduler = SCHED_RR;
  } else if (scheduler_policy == "SCHED_OTHER") {
    scheduler = SCHED_OTHER;
  }

  // --------------------------------------------------
  // 3. 创建 CIE
  // --------------------------------------------------
  // CallbackIsolatedExecutor 构造函数:
  // CallbackIsolatedExecutor(
  //   const rclcpp::ExecutorOptions &options = rclcpp::ExecutorOptions(),
  //   size_t reentrant_parallelism = 4,
  //   bool yield_before_execute = false,
  //   std::chrono::nanoseconds next_exec_timeout = std::chrono::nanoseconds(-1)
  // )

  // 使用构造函数，num_threads 作为 reentrant_parallelism 参数
  rclcpp::ExecutorOptions exec_options;
  auto executor = std::make_shared<CallbackIsolatedExecutor>(
    exec_options,           // ExecutorOptions
    num_threads,            // reentrant_parallelism (默认4)
    false,                  // yield_before_execute
    std::chrono::nanoseconds(-1)  // next_exec_timeout
  );

  // --------------------------------------------------
  // 4. 创建 ComponentManager
  // --------------------------------------------------
  auto node = std::make_shared<rclcpp_components::ComponentManager>(
    executor, "ComponentManager", options);

  executor->add_node(node);

  // 打印配置信息
  std::string affinity_str;
  for (const auto& core : cpu_affinity) {
    affinity_str += std::to_string(core) + " ";
  }

  RCLCPP_INFO(
    node->get_logger(),
    "CIE container started: "
    "num_threads=%zu, priority=%d, scheduler=%s, affinity=[%s]",
    num_threads,
    thread_priority,
    scheduler_policy.c_str(),
    affinity_str.c_str());

  // --------------------------------------------------
  // 5. spin
  // --------------------------------------------------
  executor->spin();

  rclcpp::shutdown();
  return 0;
}