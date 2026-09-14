# test.py：实验自动运行脚本说明

> 负责把某个结果目录下的 `*.launch.py` 逐一跑起来：起 LTTng trace → 起节点 →
> 预热/录制 → 停 trace → 把 trace 搬到结果目录并校验自定义事件是否录到。
> 配套文档：[`EVAL_CIE_MTE.md`](./EVAL_CIE_MTE.md)（两个实验包）、[`CIE.md`](./CIE.md)（CIE 上游安装）。

---

## 1. 在流水线中的位置

```
json2launch.py 生成 *.launch.py
        │
        ▼
test.py   （本脚本：trace + launch + 录制 + 校验）
        │
        ▼
<结果目录>/feedback_*_<时间戳>/trace   →  lttng2csv.py  → timeline CSV
```

---

## 2. 运行方式（推荐：命令行参数）

```bash
cd ~/ROSProjects/rt_eval_ws/src

# CIE（默认）
python3 test.py

# MTE（一键切到 MTE 预设目录）
python3 test.py --executor MTE

# 指定自定义目录（覆盖预设）
python3 test.py --launch-dir ./result/xxx --result-dir ./result/xxx
```

| 参数                    | 作用                           | 默认                |
| --------------------- | ---------------------------- | ----------------- |
| `--executor CIE\|MTE` | 按预设目录一键切换（`EXPERIMENT_DIRS`） | 无（用下方常量）          |
| `--launch-dir DIR`    | launch 文件所在目录                | `LAUNCH_DIR`      |
| `--result-dir DIR`    | trace/结果输出目录                 | `RESULT_BASE_DIR` |

不带任何参数 = 用脚本顶部“配置区”的常量（默认 CIE）。脚本会扫描
`<launch-dir>/*.launch.py`，逐个跑。

---

## 3. 顶部配置区（常量）

| 常量                               | 默认                            | 含义                     |
| -------------------------------- | ----------------------------- | ---------------------- |
| `LAUNCH_DIR` / `RESULT_BASE_DIR` | `./result/CIE-FIFO_IPC_nuc12` | 实验目录（也可用 CLI 覆盖）       |
| `WARMUP_TIME`                    | 5 s                           | 预热时长                   |
| `RUN_TIME`                       | 2.0 s                         | 正式录制时长                 |
| `EXPERIMENT_DIRS`                | CIE / MTE 两套预设                | `--executor` 用的目录映射    |
| `PIN_EXPERIMENT_TO_CPUS`         | `True`                        | 是否用 `taskset` 把整棵进程树钉核 |
| `CPU_OFFSET`                     | `1`                           | 钉核起始核号（绕开核 0）          |
| `ROS2_WS_PATH`                   | `~/ROSProjects/rt_eval_ws`    | source 的工作空间           |

> **钉核规则**：核数 = launch 文件名中解析出的 `m`（如 `feedback_u30_m3_...` → m=3），
> 实际钉 `CPU_OFFSET .. CPU_OFFSET+m-1`（默认 m=3 → `taskset -c 1,2,3`）。

---

## 4. 单次实验的流程（run_single_test）

对每个 launch 文件，依次执行：

1. **清理**：停掉同名残留 trace session、删 `~/.ros/tracing/<session>` 残留目录。

2. **准备子进程环境**：
   
   - 注入 `LD_PRELOAD=liblttng-ust.so`（保证 dlopen 组件库也能注册 UST 探针）；
   - `LTTNG_UST_REGISTER_TIMEOUT=-1`（探针注册不因超时丢失）。

3. **开 trace**（关键）：
   
   ```
   ros2 trace start <session> -u <默认 ros2 事件…> eval:algo_execute eval:algo_complete
   ```
   
   - 事件表 = `get_default_ust_events()`（优先从 `tracetools_trace` 读 35 个 ros2 默认事件，
     失败回退内置表）**∪** 自定义 `eval:*`。
   - ⚠️ 必须两者一起 `-u`：`-a` 只是 append，不等于“全部事件”；且 `-u` 会**替换**默认表，
     只写 `eval:*` 会丢 ros2 框架事件。

4. **启动节点**：
   
   - 若 `PIN_EXPERIMENT_TO_CPUS`，按第 3 节规则前置 `taskset -c <cores>`，
     整棵进程树（launch → cie/mte_container → 全部执行线程）钉到目标核。

5. **预热 + 录制**：sleep `WARMUP_TIME + RUN_TIME`。

6. **停 trace**：`ros2 trace stop <session>`。

7. **关节点（进程组整组清理）**：实验以独立进程组启动（`start_new_session=True`），
   结束时 `killpg` 一次杀掉 `ros2 launch` + 容器(`cie_container`/`mte_container`)
   及其子孙进程（SIGTERM → 超时 SIGKILL），杜绝残留容器污染后续 trace。

8. **搬 trace**：把 `~/.ros/tracing/<session>` 移到
   `<result-dir>/<test_name>_<HHMMSS>/trace`。

9. **校验**：`babeltrace2` 数 `eval:` 行，分三档打印：
   
   - `✅ … N 条 eval 自定义事件`
   - `⚠️ 只捕获到 ros2 默认事件，0 条 eval`
   - `❌ trace 为空`

---

## 5. 输出与产物

- 每个案例生成一个目录：`<result-dir>/feedback_*_<HHMMSS>/trace`
- 该 trace 即 `lttng2csv.py` 的输入；案例校验信息打印在终端。

---

## 6. 环境与常见问题

- **找不到 `ros2` / 包**：脚本会自动 source `ROS2_WS_PATH/install/setup.bash`
  （PATH 采用 source 后的完整值）。若仍失败，先确认该工作区已 `colcon build`。
- **CIE 容器起不来（loader 报找不到 `libcallback_isolated_executor.so`）**：
  `cie_container` 链接的是 `cie_ws` 的库，而 rt_eval_ws 的 setup 不含它。
  请在调用 shell 里先 `source ~/ROSProjects/cie_ws/install/setup.bash`。
  （MTE 不依赖 cie_ws。）
- **自定义事件录不到**：确认使用了 `-u … eval:algo_execute eval:algo_complete`
  （见第 4 步）。校验用 `eval:algo` 精确匹配，避免被 `eval::CIEComponent` 等符号名假阳性误导。
- **残留容器污染数据**：LTTng UST 是进程级全局录制，上一轮没关干净的 `cie_container`/`mte_container`
  会把它的负载混进本轮 trace（表现为出现额外 tid）。已改为每轮结束按进程组整组清理，
  如需手动清：`pkill -x cie_container; pkill -x mte_container`。
- **权限**：钉核(`taskset`)无需特权；若需 SCHED_FIFO/RT 由 `cie_thread_configurator` 完成
  （需 `cap_sys_nice`，见 CIE.md）。

---

**文档版本**：1.0
**更新日期**：2026-09-08
**适用 ROS 2 版本**：Jazzy
