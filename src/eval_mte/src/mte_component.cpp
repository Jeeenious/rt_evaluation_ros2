#define TRACEPOINT_DEFINE
#define TRACEPOINT_CREATE_PROBES
#include "mte_tracepoints.h"

#include <chrono>
#include <vector>
#include <deque>
#include <string>
#include <regex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

namespace eval {

class MTEComponent : public rclcpp::Node {
public:
    // 构造函数参数必须符合组件规范
    explicit MTEComponent(const rclcpp::NodeOptions & options)
    : Node("mte_task", options), node_id_("") {
        this->declare_parameter("wcet_us", 0);
        this->declare_parameter("period_ms", 0);
        this->declare_parameter("input_topics", std::vector<std::string>{});
        this->declare_parameter("output_topics", std::vector<std::string>{});
        this->declare_parameter("node_id", "");  // string 类型，默认为空

        wcet_us_ = this->get_parameter("wcet_us").as_int();
        node_id_ = this->get_parameter("node_id").as_string();  // 读取 string
        int period_ms = this->get_parameter("period_ms").as_int();
        auto inputs = this->get_parameter("input_topics").as_string_array();
        auto outputs = this->get_parameter("output_topics").as_string_array();

        callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        for (const auto& t : outputs) {
            pubs_.push_back(this->create_publisher<std_msgs::msg::String>(t, 10));
        }

        if (period_ms > 0) {
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(period_ms),
                std::bind(&MTEComponent::do_work, this),
                callback_group_);
        } else {
            for (const auto& t : inputs) {
                rclcpp::SubscriptionOptions options;
                options.callback_group = callback_group_;

                subs_.push_back(this->create_subscription<std_msgs::msg::String>(
                    t, 10, [this](std_msgs::msg::String::SharedPtr msg) {
                        (void)msg;
                        this->do_work();
                    }, options));
            }
        }
    }

private:

    inline long long thread_cpu_time_us()
    {
        struct timespec ts{};

        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);

        return static_cast<long long>(ts.tv_sec) * 1'000'000LL
          + static_cast<long long>(ts.tv_nsec) / 1'000LL;
    }

    void spin_cost_us(long long us)
    {
        if (us <= 0)
            return;

        const long long start_cpu_us = thread_cpu_time_us();

        long long segment_start_cpu_us = start_cpu_us;
        int segment_cpu = sched_getcpu();

        while (thread_cpu_time_us() - start_cpu_us < us)
        {
            const long long current_cpu_us = thread_cpu_time_us();
            const int current_cpu = sched_getcpu();

            if (current_cpu != segment_cpu)
            {
                const long long segment_cpu_us =
                  current_cpu_us - segment_start_cpu_us;

                tracepoint(eval, algo_working,
                           node_id_.c_str(),
                           segment_cpu,
                           segment_cpu_us);

                segment_cpu = current_cpu;
                segment_start_cpu_us = current_cpu_us;
            }
        }

        // while 结束，记录最后一段
        const long long end_cpu_us = thread_cpu_time_us();
        const long long segment_cpu_us =
            end_cpu_us - segment_start_cpu_us;

        tracepoint(
            eval, algo_working,
            node_id_.c_str(),
            segment_cpu,
            segment_cpu_us);
    }

    void do_work()
    {
        tracepoint(eval, algo_execute,
                   node_id_.c_str());

        spin_cost_us(wcet_us_);

        tracepoint(eval, algo_complete,
                   node_id_.c_str());

        auto msg = std_msgs::msg::String();
        msg.data = "payload";
        for (auto& pub : pubs_)
        {
            pub->publish(msg);
        }
    }

    int wcet_us_;
    std::string node_id_;
    std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> pubs_;
    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subs_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace eval

// 注册为组件
RCLCPP_COMPONENTS_REGISTER_NODE(eval::MTEComponent);