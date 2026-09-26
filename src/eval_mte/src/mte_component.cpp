#define TRACEPOINT_DEFINE
#define TRACEPOINT_CREATE_PROBES
#include "mte_tracepoints.h"

#include <algorithm>
#include <chrono>
#include <vector>
#include <deque>
#include <mutex>
#include <string>
#include <regex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

namespace eval {

class MTEComponent : public rclcpp::Node {
public:
    // payload 大小默认值：构造大消息用于压 IPC 路径。它的 CPU 时间由 do_work 记账。
    // 实际大小可由 payload_bytes 参数覆盖（对应 JSON 的 configs[2] / 文件名 msg<N>）。
    static constexpr std::size_t DEFAULT_PAYLOAD_BYTES = 1024 * 1024;

    // 构造函数参数必须符合组件规范
    explicit MTEComponent(const rclcpp::NodeOptions & options)
    : Node("mte_task", options), node_id_("") {
        this->declare_parameter("wcet_us", 0);
        this->declare_parameter("period_ms", 0);
        this->declare_parameter("input_topics", std::vector<std::string>{});
        this->declare_parameter("output_topics", std::vector<std::string>{});
        this->declare_parameter("node_id", "");  // string 类型，默认为空
        this->declare_parameter("trigger_topics", std::vector<std::string>{});
        this->declare_parameter("trigger_counts", std::vector<int64_t>{});
        this->declare_parameter("acc_topic", std::string(""));
        this->declare_parameter("acc_window", 0);
        this->declare_parameter("payload_bytes",
            static_cast<int64_t>(DEFAULT_PAYLOAD_BYTES));

        wcet_us_ = this->get_parameter("wcet_us").as_int();
        node_id_ = this->get_parameter("node_id").as_string();  // 读取 string
        int period_ms = this->get_parameter("period_ms").as_int();
        auto outputs = this->get_parameter("output_topics").as_string_array();

        const auto trigger_topics = this->get_parameter("trigger_topics").as_string_array();
        const auto trigger_counts = this->get_parameter("trigger_counts").as_integer_array();
        acc_topic_ = this->get_parameter("acc_topic").as_string();
        acc_window_ = this->get_parameter("acc_window").as_int();

        const int64_t payload_bytes = this->get_parameter("payload_bytes").as_int();
        if (payload_bytes > 0) {
            payload_bytes_ = static_cast<std::size_t>(payload_bytes);
        } else {
            RCLCPP_WARN(this->get_logger(),
                "节点 '%s' 的 payload_bytes=%ld 非法，回退默认 %zu",
                node_id_.c_str(), static_cast<long>(payload_bytes), DEFAULT_PAYLOAD_BYTES);
        }

        // 每个触发端口的稀释因子：需到达 trigger_need_[i] 次，后级才执行一次
        if (!trigger_counts.empty() && trigger_counts.size() != trigger_topics.size()) {
            RCLCPP_ERROR(this->get_logger(),
                "节点 '%s' 的 trigger_counts(%zu) 与 trigger_topics(%zu) 长度不一致",
                node_id_.c_str(), trigger_counts.size(), trigger_topics.size());
        }
        trigger_need_.assign(trigger_topics.size(), 1);
        for (std::size_t i = 0; i < trigger_topics.size() && i < trigger_counts.size(); ++i) {
            if (trigger_counts[i] > 0) {
                trigger_need_[i] = static_cast<int>(trigger_counts[i]);
            }
        }

        // 触发模式由 period_ms 唯一确定：>0 定时器，=0 订阅触发
        const bool is_timer = (period_ms > 0);

        callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        for (const auto& t : outputs) {
            pubs_.push_back(this->create_publisher<std_msgs::msg::String>(t, 10));
        }

        if (is_timer) {
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(period_ms),
                std::bind(&MTEComponent::do_work, this),
                callback_group_);
        } else {
            // 只有 trigger_topics 触发执行：每个端口攒够自己的稀释因子，
            // 且所有端口都攒够，才算一轮触发。
            if (trigger_topics.empty()) {
                RCLCPP_ERROR(this->get_logger(),
                    "event 节点 '%s' 的 trigger_topics 为空，永远不会执行",
                    node_id_.c_str());
            }

            trigger_seen_.assign(trigger_topics.size(), 0);

            for (std::size_t i = 0; i < trigger_topics.size(); ++i) {
                rclcpp::SubscriptionOptions options;
                options.callback_group = callback_group_;

                subs_.push_back(this->create_subscription<std_msgs::msg::String>(
                    trigger_topics[i], 10, [this, i](std_msgs::msg::String::SharedPtr msg) {
                        (void)msg;
                        this->on_trigger(i);
                    }, options));
            }
        }

        // ------------------------------------------------------------
        // acc 主题（hist）：只维护"最近 acc_window 个样本"的输入窗口，
        // 本身不触发执行。周期节点同样可以带 hist。
        // ------------------------------------------------------------
        if (!acc_topic_.empty() && acc_window_ > 0) {
            rclcpp::SubscriptionOptions options;
            options.callback_group = callback_group_;

            acc_sub_ = this->create_subscription<std_msgs::msg::String>(
                acc_topic_, 10, [this](std_msgs::msg::String::SharedPtr msg) {
                    this->on_accumulate(msg);
                }, options);
        }
    }

private:
    /*
     * 触发回调。每个触发端口维护一个到达计数，攒够各自的稀释因子后
     * （trigger_seen_[i] >= trigger_need_[i] 对所有端口成立）才执行一次，
     * 随后全部清零重新计数：
     *   - 单路 event、稀释因子 1：每次到达都触发
     *   - 稀释因子 n：该端口到达 n 次才放行一次（n 次输入对应 1 次输出）
     *   - 多路 event（join）：各路都攒够，相当于对各路取汇合
     * 计数在锁内清零后才释放锁，因此并发到达时只有最后一个线程会执行 do_work。
     */
    void on_trigger(std::size_t idx) {
        bool fire = false;

        {
            std::lock_guard<std::mutex> lock(trigger_mutex_);

            if (trigger_seen_[idx] < trigger_need_[idx]) {
                ++trigger_seen_[idx];
            }

            fire = std::equal(
                trigger_seen_.begin(), trigger_seen_.end(), trigger_need_.begin(),
                [](int seen, int need) { return seen >= need; });

            if (fire) {
                std::fill(trigger_seen_.begin(), trigger_seen_.end(), 0);
            }
        }

        if (fire) {
            do_work();
        }
    }

    /*
     * acc 主题回调：滑动窗口，只保留最近 acc_window_ 个样本。
     * 这是节点的输入状态，不触发执行，也不被 do_work 消费。
     */
    void on_accumulate(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(acc_mutex_);

        acc_buf_.push_back(msg);

        while (acc_buf_.size() > static_cast<std::size_t>(acc_window_)) {
            acc_buf_.pop_front();
        }
    }


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

        // 任务开始时的线程 CPU 时间
        const long long start_cpu_us = thread_cpu_time_us();

        // 当前 execution segment 的起点
        long long segment_start_cpu_us = start_cpu_us;

        // 当前所在 CPU
        int segment_cpu = sched_getcpu();

        while (true)
        {
            /*
             * CLOCK_THREAD_CPUTIME_ID 只统计当前线程真正获得 CPU 的
             * 时间。因此：
             *
             *   A running  -> CPU time 增加
             *   A 被抢占  -> CPU time 不增加
             *
             * 这正好可以用来模拟 WCET。
             */
            const long long current_cpu_us = thread_cpu_time_us();

            const long long total_cpu_us =
                current_cpu_us - start_cpu_us;

            // ------------------------------------------------------------
            // 1. 检查任务是否已经获得足够的 CPU execution time
            // ------------------------------------------------------------
            if (total_cpu_us >= us)
            {
                /*
                 * 当前 segment 实际只需要记录到 us。
                 *
                 * 不记录循环采样粒度带来的超出量，只记录剩余预算，
                 * 这样所有 algo_working 段之和恰好等于 us。
                 */
                const long long segment_cpu_us =
                    us - (segment_start_cpu_us - start_cpu_us);

                if (segment_cpu_us > 0)
                {
                    tracepoint(
                        eval,
                        algo_working,
                        node_id_.c_str(),
                        segment_cpu,
                        segment_cpu_us);
                }

                break;
            }

            // ------------------------------------------------------------
            // 2. 检查 CPU 是否发生 migration
            // ------------------------------------------------------------
            const int current_cpu = sched_getcpu();

            if (current_cpu != segment_cpu)
            {
                /*
                 * 当前 segment 的长度使用 THREAD_CPUTIME 计算。
                 *
                 * 注意：
                 * 如果期间发生了同核抢占，这里的 CPU time 不会增长，
                 * 因此不会把被抢占的时间错误地算进 execution time。
                 */
                const long long segment_cpu_us =
                    current_cpu_us - segment_start_cpu_us;

                if (segment_cpu_us > 0)
                {
                    tracepoint(
                        eval,
                        algo_working,
                        node_id_.c_str(),
                        segment_cpu,
                        segment_cpu_us);
                }

                // 开始新的 execution segment
                segment_cpu = current_cpu;
                segment_start_cpu_us = current_cpu_us;
            }
        }
    }

    void do_work()
    {
        tracepoint(eval, algo_execute,
                   node_id_.c_str());

        // ------------------------------------------------------------
        // 1. 真实工作负载：构造 payload（1MB 分配 + 填充）
        //
        //    这段是真实的 CPU 开销，必须记账：一方面用 THREAD_CPUTIME
        //    量出它到底消耗了多少 CPU，作为一段 algo_working 发出，
        //    否则这期间发生的抢占/迁移在 trace 里无法归属；另一方面
        //    它要占用 wcet 预算（见第 2 步），使节点总 CPU 消耗恰好
        //    等于模型给出的 wcet_us。
        // ------------------------------------------------------------
        const long long work_start_cpu_us = thread_cpu_time_us();

        auto msg = std_msgs::msg::String();
        msg.data.assign(payload_bytes_, 'x');

        const long long work_cpu_us = thread_cpu_time_us() - work_start_cpu_us;

        if (work_cpu_us > 0)
        {
            tracepoint(eval, algo_working,
                       node_id_.c_str(),
                       sched_getcpu(),
                       work_cpu_us);
        }

        // ------------------------------------------------------------
        // 2. 忙等补足剩余预算：总预算 wcet_us 减去真实工作已用掉的部分。
        //    若真实工作已超出 wcet_us，spin_cost_us 直接返回（不再补）。
        // ------------------------------------------------------------
        spin_cost_us(wcet_us_ - work_cpu_us);

        tracepoint(eval, algo_complete,
                   node_id_.c_str());

        for (auto& pub : pubs_)
        {
            pub->publish(msg);
        }
    }

    int wcet_us_;
    int acc_window_ = 0;
    std::size_t payload_bytes_ = DEFAULT_PAYLOAD_BYTES;
    std::string node_id_;
    std::string acc_topic_;
    std::vector<int> trigger_seen_;
    std::vector<int> trigger_need_;
    std::mutex trigger_mutex_;
    std::mutex acc_mutex_;
    std::deque<std_msgs::msg::String::SharedPtr> acc_buf_;
    std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> pubs_;
    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subs_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr acc_sub_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace eval

// 注册为组件
RCLCPP_COMPONENTS_REGISTER_NODE(eval::MTEComponent);