# eval_mte：MTE 执行器实验

本文档说明 `eval_mte` 功能包的用途、组成、运行链路，以及自定义 LTTng trace 事件 `eval:*` 的使用方法。

---

## 1. 实验目的

`eval_mte` 用于在 ROS 2 `MultiThreadedExecutor` 上运行任务图，并通过 LTTng 记录每个节点的算法执行时间。

任务图通常为：

```text
nn0 → nn1 → nn2 → nn3 → nn4 → nn5 → nn6 → nn7
```

节点之间通过 ROS 2 topic 传递消息。

实验主要关注：

- callback 执行时间

- 节点间端到端延迟

- executor 调度行为

- callback 并发与竞争

- MTE 与 CIE 的性能差异

---

## 2. 功能包组成

| 项目              | 内容                      |
| --------------- | ----------------------- |
| Package         | `eval_mte`              |
| Component       | `eval::MTEComponent`    |
| Container       | `mte_container`         |
| Node executable | `mte_node_exec`         |
| Executor        | `MultiThreadedExecutor` |

`MTEComponent` 与 `CIEComponent` 的业务逻辑保持一致，仅 executor 不同，因此可以进行公平对比。

---

## 3. Component 行为

组件由以下参数驱动：

- `wcet_us`：模拟计算时间

- `period_ms`：周期触发时间；`>0` 使用 timer，`=0` 使用 subscription

- `input_topics`

- `output_topics`

- `node_id`

每个组件使用 Reentrant callback group。

核心执行流程：

```text
callback
   │
   ├── tracepoint(eval, algo_execute, node_id)
   │
   ├── spin_cost_us(wcet_us)
   │
   ├── publish output
   │
   └── tracepoint(eval, algo_complete, node_id)
```

`algo_execute` 和 `algo_complete` 分别用于标记一次算法执行的开始和结束。

---

## 4. LTTng Tracepoint

文件：

```text
eval_mte/include/mte_tracepoints.h
```

使用：

```text
TRACEPOINT_PROVIDER = eval
```

事件：

```text
eval:algo_execute
eval:algo_complete
```

事件字段：

```text
node_id
```

由于 CIE 和 MTE 使用相同的 provider/event 名称，因此后处理脚本可以统一解析：

```text
eval:algo_execute
eval:algo_complete
```

---

## 5. 实验运行链路

```text
*.json
   │
   ▼
json2launch.py
   │
   ▼
*.launch.py
   │
   ▼
mte_container
   │
   ├── MTEComponent × 8
   │
   ▼
LTTng trace
   │
   ▼
lttng2csv.py
   │
   ▼
xxx_timeline.csv
```

主要脚本：

| 文件                   | 作用                         |
| -------------------- | -------------------------- |
| `src/json2launch.py` | 根据任务图生成 launch             |
| `src/test.py`        | 自动启动 tracing、运行实验并保存 trace |
| `src/lttng2csv.py`   | 将 CTF trace 转换为时间线 CSV     |

---

## 6. 编译环境

```bash
cd ~/ROSProjects/rt_eval_ws

colcon build --packages-select eval_mte
```

运行前：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ROSProjects/rt_eval_ws/install/setup.bash
```

检查组件：

```bash
ros2 component types | grep eval
```

---

## 7. 使能 `eval:*` Trace

`ros2 trace start` 不会自动记录自定义 `eval:*` 事件。

必须显式加入：

```bash
eval:algo_execute
eval:algo_complete
```

同时保留需要的 ROS 2 tracing events，例如：

```bash
ros2 trace start <session> -u \
    ros2:callback_start \
    ros2:callback_end \
    ros2:rclcpp_executor_execute \
    ros2:rclcpp_executor_wait_for_work \
    ros2:rclcpp_executor_get_next_ready \
    eval:algo_execute \
    eval:algo_complete
```

项目中的 `test.py` 会读取 `tracetools_trace` 默认 ROS 2 events，再追加：

```text
eval:algo_execute
eval:algo_complete
```

---

## 8. 验证 Trace

实验结束后：

```bash
ros2 trace stop
```

检查：

```bash
babeltrace2 <trace_dir> | grep -c 'eval:algo'
```

查看具体事件：

```bash
babeltrace2 <trace_dir> | grep 'eval:algo'
```

正常情况下应该可以看到：

```text
eval:algo_execute
eval:algo_complete
```

---

## 9. 与 Scheduler Trace 配合

如果需要分析 callback 的 CPU 调度行为，可以同时记录：

```text
sched_waking
sched_wakeup
sched_switch
```

推荐时间线：

```text
callback ready
      │
      ▼
sched_wakeup
      │
      ▼
sched_switch
      │
      ▼
callback_start
      │
      ▼
callback_end
```

其中 `sched_switch` 用于判断 executor thread 之间的 CPU 切换。

---

## 10. 已知注意事项

生成 MTE launch 时，确认组件名称为：

```text
eval::MTECo
```
