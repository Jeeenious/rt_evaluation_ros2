# eval_cie 与 eval_mte：实验功能包说明

> 本文档说明 `eval_cie`（CIE 执行器实验）与 `eval_mte`（MTE 执行器实验）两个功能包的用途、
> 组成、运行链路，以及自定义 LTTng trace 事件（`eval:*`）的监控方法。
> 配套文档：[`CIE.md`](./CIE.md)（上游 CallbackIsolatedExecutor 的安装与权限配置）。

---

## 1. 这两个包是做什么的

用于做 ROS 2 **执行器（executor）对比实验**：把同一份任务图分别跑在 `CallbackIsolatedExecutor`（CIE）与 `MultiThreadedExecutor`（MTE）上，用 LTTng 录下
每个节点“算法执行”的自定义事件，量化端到端延迟 / 调度行为。

| 包          | 组件(plugin)           | 容器可执行文件                             | 使用的执行器                              |
| ---------- | -------------------- | ----------------------------------- | ----------------------------------- |
| `eval_cie` | `eval::CIEComponent` | `cie_container`（另有 `cie_node_exec`） | `CallbackIsolatedExecutor`（回调隔离执行器） |
| `eval_mte` | `eval::MTEComponent` | `mte_container`（另有 `mte_node_exec`） | `MultiThreadedExecutor`（标准多线程执行器）   |

两个组件的业务逻辑**完全一样**（同一份任务图），只是 executor 不同，因此可用于公平对比。

### 1.1 组件行为（CIEComponent / MTEComponent）

- 由参数驱动：`wcet_us`（忙等耗时）、`period_ms`（>0 走定时器，=0 走订阅触发）、 `input_topics` / `output_topics`、`node_id`。
- 每个组件把定时器/订阅回调放进一个 **Reentrant 回调组**。
- `do_work()` 的执行流：
  - `tracepoint(eval, algo_execute, node_id)` —— 开始
  - `spin_cost_us(wcet_us)` —— 模拟计算负载（忙等）
  - 向 `output_topics` 发布消息
  - `tracepoint(eval, algo_complete, node_id)` —— 结束
    （CIE 与 MTE 在“发布”与 `algo_complete` 的相对顺序上略有差异。）

任务图是链式的：`node_nn0 → nn1 → … → nn7`，前驱发布、后继订阅。通常 `nn0/nn1/nn3/nn4` 等节点有 `period_ms>0`（周期源），其余为订阅触发。

### 1.2 自定义 tracepoint

两个包各自定义 LTTng-UST 探针，但**提供者同名、事件同名**（设计如此，便于统一解析）：

| 头文件                                  | TRACEPOINT_PROVIDER | 事件                              |
| ------------------------------------ | ------------------- | ------------------------------- |
| `eval_cie/include/cie_tracepoints.h` | `eval`              | `algo_execute`, `algo_complete` |
| `eval_mte/include/mte_tracepoints.h` | `eval`              | `algo_execute`, `algo_complete` |

事件字段：`node_id`（string）。解析脚本 `_lttng2timeline.py` 用 `eval:algo_execute` / `eval:algo_complete` 来标记每一轮“算法执行”。

---

## 2. 运行链路（一个实验怎么跑起来的）

```
*.json(任务图)
   │  json2launch.py（EXECUTOR_TYPE = CIE / MTE 切换）
   ▼
*.launch.py  (ComposableNodeContainer：动态 load 8 个组件进容器)
   │  test.py（先 ros2 trace start，再 ros2 launch）
   ▼
LTTng trace (含 ros2:* 框架事件 + eval:* 自定义事件)
   │  lttng2csv.py（bt2 解析）
   ▼
xxx_timeline.csv  (列：tid, seq, kind, t_us, cpu, tag)
```

| 脚本                   | 作用                                                                                                   |
| -------------------- | ---------------------------------------------------------------------------------------------------- |
| `src/json2launch.py` | 把 json 任务图生成 `ComposableNodeContainer` 版 launch；`EXECUTOR_TYPE` 常量切 CIE/MTE，分别引用对应包/组件/容器。           |
| `src/test.py`        | 自动化跑一个 launch：起 trace → 起节点 → 预热+录制 → 停 trace → 把 `~/.ros/tracing/<session>` 移到结果目录 → babeltrace 校验。 |
| `../_lttng2timeline.py`   | 从 trace 目录解析事件时间线 CSV（自定义事件与 ros2 框架事件对齐）。                                                           |

结果目录（`src/result/` 下按实验组织）：`CIE-FIFO_IPC_nuc12` / `MTE_!IPC_nuc12` / `MTE_IPC_nuc12` 等，每个 run 一个 `feedback_*_<时间戳>/trace`。

---

## 3. 编译 / 环境

两个包依赖：`rclcpp`、`rclcpp_components`、`std_msgs`、`lttng-ust`；`eval_cie` 额外依赖 `callback_isolated_executor`（来自 `cie_ws` 工作区）。

```bash
# 先建好上游 CIE 工作区（见 doc/CIE.md），然后编译本工作区
cd ~/ROSProjects/cie_ws && colcon build --symlink-install
cd ~/ROSProjects/rt_eval_ws && colcon build --packages-select eval_cie eval_mte

# 运行前必须把两个工作区都 source（顺序不限，需都生效）
source ~/ROSProjects/cie_ws/install/setup.bash
source ~/ROSProjects/rt_eval_ws/install/setup.bash
```

> ⚠️ **关键**：`cie_container` 链接 `libcallback_isolated_executor.so`，该库只在 `cie_ws`。
> 若只 source rt_eval_ws，`cie_container` 会因“找不到共享库”直接起不来（`ldd` 可见 not found）。
> 用 `test.py` 在干净 shell 下自动加载环境时不会补 cie_ws，需在调用 shell 里先 source。

---

## 4. 监控自定义 trace 事件（务必使能 `eval:*`）

### 4.1 正确姿势

`ros2 trace start <session>` **不会**录到自定义事件，因为默认只使能 ros2_tracing 内置的
35 个 `ros2:*` 事件。且注意：`ros2 trace start` 的 `-a` 是 `--append-trace`（目录已存在时
允许追加），**不是**“启用所有事件”。

必须显式把事件表传进去，且要 **默认 ros2 事件 ∪ eval 事件**（`-u` 会替换默认表而不是追加，
只写 `eval:*` 会丢掉 ros2 框架事件，导致 lttng2csv 无法对齐）：

```bash
ros2 trace start <session> -u \
  ros2:callback_start ros2:callback_end ros2:rclcpp_executor_execute \
  ros2:rclcpp_executor_wait_for_work ros2:rclcpp_executor_get_next_ready \
  eval:algo_execute eval:algo_complete   # 再加其它需要的 ros2 事件
```

`test.py` 已实现：`get_default_ust_events()` 优先从 `tracetools_trace` 读取默认 35 事件，
再拼上 `eval:algo_execute` / `eval:algo_complete`。

### 4.2 校验

```bash
babeltrace2 <trace_dir> | grep -c 'eval:algo'
```

或看 trace 的 CTF metadata 里是否声明了 `eval:algo_execute`（声明了 = 事件已被使能并注册）。

> 排查提示：`grep 'eval:'` 可能命中 `eval::CIEComponent` 等符号（假阳性），
> 用 `eval:algo` 精确匹配。

---

## 5. 本轮排障记录（重要结论）

现象：trace 里只有 `ros2:*`，`eval:*` 永远是 0。原因是两层问题叠加：

1. **使能层**：事件没被 `-u` 使能（见第 4 节）。—— 已修复 `test.py`。
2. **运行层（CIE 特有）**：`CallbackIsolatedExecutor::spin()` 只给 spin 开始时已注册的
   回调组开线程；而 launch 的 `ComposableNodeContainer` 是 spin 之后才经 `ComponentManager::load_node` 动态把组件塞进 executor，导致这些组件的定时器/订阅 **根本没有执行线程**，`do_work` 从不运行，`eval` 事件自然为 0（连 ros2 回调也没有稳态出现）。
   MTE 用标准 `MultiThreadedExecutor`（能响应运行期 `add_node`），所以无此问题。
   —— 已修复 `cie_ws` 的 `CallbackIsolatedExecutor`：`spin()` 改为 dispatcher 主循环，
   持续为 spin 期间动态加入的节点/回调组启动执行线程。

修复后，同一 CIE 案例端到端能录到 ~586 次 `eval:algo_execute` / `algo_complete` （8 节点各 ~65 次，50ms 的 `node_nn3` 约 131 次），与 MTE 表现一致。

### 其它已知注意事项

- `cie_container` 当前只把 `num_threads` 用作执行器并行度；`thread_priority`、 `scheduler_policy`、`cpu_affinity` 会被解析/打印但**未真正应用**（未调用 `sched_setscheduler`/亲和性设置）。需要实时调度时请配合 `cie_thread_configurator`（见 CIE.md）。
- `test.py` 结束实验时 `terminate()` 后容器子进程偶尔残留，长时间批量跑建议自行加固回收。
- 生成 MTE launch 前，确认 `json2launch.py` 中 MTE 组件名为 `eval::MTEComponent` （旧配置曾误写为 `eval::SimComponent`，会导致加载失败）。

---

**文档版本**：1.0 **更新日期**：2026-09-08 **适用 ROS 2 版本**：Jazzy
