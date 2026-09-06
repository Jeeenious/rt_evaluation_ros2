import json
import os
import glob
import re

# ================= 配置区 =================
INPUT_DIR = "./pipelines"
OUTPUT_DIR = "./launches"
PACKAGE_NAME = "eval"
COMPONENT_NAME = "eval::SimComponent"

# 组件容器所在包与可执行文件（自定义容器 usr_container 会读 num_threads 参数，
# 使文件名中的 m 真正决定 executor 线程数）
CONTAINER_PACKAGE = "eval"
CONTAINER_EXECUTABLE = "usr_container"

# ====== Executor 配置 ======
# 可选值:
#   - "MultiThreadedExecutor"      # ROS 2 标准多线程执行器
#   - "SingleThreadedExecutor"     # ROS 2 标准单线程执行器（不支持）
#   - "EventsExecutor"             # ROS 2 实验性事件驱动执行器（不支持）
#   - "CallbackIsolatedExecutor"   # Autoware 的回调隔离执行器（每个回调组独立线程）
# 线程配置（仅对 MultiThreadedExecutor 和 CallbackIsolatedExecutor 有效）
# - MultiThreadedExecutor: 线程池大小
# - CallbackIsolatedExecutor: 每个回调组独立线程，此值用于限制最大并发数
EXECUTOR_TYPE = "MultiThreadedExecutor"

# USE_INTRAPROCESS: 是否启用 ROS 2 内部进程通信（intra-process communication）
USE_INTRAPROCESS = True

# ====== CallbackIsolatedExecutor 专用配置 ======
# CIE 线程配置（如果未提供配置文件，则使用这些默认值）
CIE_THREAD_PRIORITY = 50                # 线程优先级 (0-99, 仅对 SCHED_FIFO/RR 有效)
CIE_THREAD_SCHED_POLICY = "SCHED_FIFO"  # 可选: "SCHED_FIFO", "SCHED_RR", "SCHED_OTHER"
CIE_CPU_AFFINITY = []                   # CPU 亲和性，例如 [0, 1] 表示绑定到 CPU 0 和 1
# ===========================================

# ====== QoS 配置 ======
QOS_RELIABILITY = "reliable"
QOS_HISTORY = "keep_last"
QOS_DEPTH = 10
QOS_DURABILITY = "volatile"
QOS_DEADLINE = 0
QOS_LIFESPAN = 0
# ==========================

def parse_m_from_filename(filename):
    match = re.search(r'_m(\d+)', filename)
    if match:
        return int(match.group(1))
    else:
        match = re.search(r'm(\d+)_', filename)
        if match:
            return int(match.group(1))
    print(f"  [Warning] 无法从 {filename} 解析 m 值，使用默认值 1")
    return 1

def get_executor_config(m_value):
    """根据 m 值和执行器类型返回配置"""
    config = {
        "threads": 1,
        "note": "",
        "executable": "component_container"
    }

    if EXECUTOR_TYPE == "SingleThreadedExecutor":
        config["threads"] = 1
        config["note"] = f"单线程执行器，m={m_value} 被忽略"
        config["executable"] = "component_container"

    elif EXECUTOR_TYPE == "MultiThreadedExecutor":
        threads = m_value if m_value > 0 else os.cpu_count()
        config["threads"] = threads
        config["note"] = f"多线程执行器，使用 {threads} 个线程 (m={m_value})"
        config["executable"] = CONTAINER_EXECUTABLE  # usr_container 按 num_threads 起线程

    elif EXECUTOR_TYPE == "EventsExecutor":
        config["threads"] = 1  # 目前官方版本是单线程
        config["note"] = f"事件驱动执行器（实验性），目前为单线程，m={m_value} 被忽略"
        config["executable"] = "component_container"  # 使用普通容器

    elif EXECUTOR_TYPE == "CallbackIsolatedExecutor":
        threads = m_value if m_value > 0 else os.cpu_count()
        config["threads"] = threads
        config["note"] = f"回调隔离执行器 (CIE)，每个回调组独立线程，最大并发数 {threads} (m={m_value})"
        config["executable"] = "component_container_mt"  # CIE 基于多线程容器
        config["cie_enabled"] = True

    else:
        # 默认回退到 MultiThreadedExecutor
        threads = m_value if m_value > 0 else os.cpu_count()
        config["threads"] = threads
        config["note"] = f"未知执行器类型，回退到多线程执行器，使用 {threads} 个线程"
        config["executable"] = CONTAINER_EXECUTABLE  # 同上，按 num_threads 起线程

    return config

def _fmt_number(value):
    """整数值按 int 输出。

    sim_node.cpp 的参数用 declare_parameter(x, 0) 声明为整型；若生成的 launch
    里写成 100.0（double），组件加载时会被判为类型不匹配而抛异常、节点起不来。
    这里把整数形式的浮点归一成 int。"""
    if isinstance(value, float) and value.is_integer():
        return int(value)
    return value


def fins_json_to_component_launch(json_path, launch_path):
    try:
        with open(json_path, 'r') as f:
            pipeline = json.load(f)
    except Exception as e:
        print(f"  [Error] Failed to parse {json_path}: {e}")
        return

    filename = os.path.basename(json_path)
    m_value = parse_m_from_filename(filename)
    executor_config = get_executor_config(m_value)

    print(f"  [m解析] {filename} -> m={m_value}")
    print(f"  [执行器] {EXECUTOR_TYPE} -> {executor_config['note']}")

    # 构建 Launch 脚本内容
    content = [
        "import launch",
        "from launch_ros.actions import ComposableNodeContainer",
        "from launch_ros.descriptions import ComposableNode",
        "",
        "# ===== Executor 配置 =====",
        f"# 类型: {EXECUTOR_TYPE}",
        f"# 文件名: {filename}",
        f"# m值: {m_value}",
        f"# 说明: {executor_config['note']}",
        "# =========================",
        "",
        "# ===== QoS 配置 =====",
        f"# Reliability: {QOS_RELIABILITY}",
        f"# History: {QOS_HISTORY}",
        f"# Depth: {QOS_DEPTH}",
        f"# Durability: {QOS_DURABILITY}",
        f"# Deadline: {QOS_DEADLINE}ms",
        f"# Lifespan: {QOS_LIFESPAN}ms",
        "# ===================",
        "",
        "def generate_launch_description():",
    ]

    # 根据执行器类型添加不同的导入和配置
    if EXECUTOR_TYPE == "CallbackIsolatedExecutor":
        content.extend([
            "    # 使用 CallbackIsolatedExecutor (CIE)",
            "    # 需要安装: https://github.com/autowarefoundation/callback_isolated_executor",
            "    try:",
            "        from callback_isolated_executor import create_container_with_cie",
            "        use_cie = True",
            "    except ImportError:",
            "        print('Warning: callback_isolated_executor not found, falling back to standard container')",
            "        use_cie = False",
            "",
            "    # 如果没有 CIE，使用标准多线程容器",
            "    if use_cie:",
            "        # CIE 需要配置线程参数",
            "        cie_config = {",
            "            'thread_priority': {CIE_THREAD_PRIORITY},",
            f"            'sched_policy': '{CIE_THREAD_SCHED_POLICY}',",
            f"            'cpu_affinity': {CIE_CPU_AFFINITY if CIE_CPU_AFFINITY else 'None'},",
            "            'max_concurrent_callbacks': {executor_config['threads']}",
            "        }",
            "        container = create_container_with_cie(",
            "            name='fins_eval_container',",
            "            namespace='',",
            "            package='rclcpp_components',",
            f"            executable='{executor_config['executable']}',",
            "            composable_node_descriptions=[",
        ])
    else:
        # 标准执行器
        content.extend([
            f"    # {EXECUTOR_TYPE}",
            "    container = ComposableNodeContainer(",
            "        name='fins_eval_container',",
            "        namespace='',",
            f"        package='{CONTAINER_PACKAGE}',",
            f"        executable='{executor_config['executable']}',",
            f"        parameters=[{{'num_threads': {executor_config['threads']}}}],",
            "        composable_node_descriptions=[",
        ])

    # 生成节点配置
    for node_cfg in pipeline:
        node_id = node_cfg.get("id", "unknown")

        wcet_us = 0
        if "parameters" in node_cfg and len(node_cfg["parameters"]) > 0:
            wcet_us = _fmt_number(node_cfg["parameters"][0].get("value", 0))

        period_ms = _fmt_number(node_cfg.get("period", 0))
        inputs = node_cfg.get("inputs", [])
        outputs = node_cfg.get("outputs", [])

        acc_window = 0
        if "loop" in node_cfg and node_cfg["loop"]:
            acc_window = list(node_cfg["loop"].values())[0]

        # 为 CIE 添加回调组配置
        if EXECUTOR_TYPE == "CallbackIsolatedExecutor":
            callback_group = """        callback_group='callback_group',
        callback_group_kwargs={'mutually_exclusive': False},
"""
        else:
            callback_group = ""

        # input/output topics 只在非空时才写进参数表。
        # launch 解析到空列表会当成空 tuple，报 "Expected 'value' ... got '()'" 中止；
        # sim_node.cpp 已用 declare_parameter 声明默认空数组，留空即省略键，语义不变。
        _tind = "                        "
        topics_block = ""
        if inputs:
            topics_block += f"{_tind}'input_topics': {inputs!r},\n"
        if outputs:
            topics_block += f"{_tind}'output_topics': {outputs!r},\n"

        node_entry = f"""            ComposableNode(
                package='{PACKAGE_NAME}',
                plugin='{COMPONENT_NAME}',
                name='node_n{node_id}',
                parameters=[
                    {{
                        'wcet_us': {wcet_us},
                        'period_ms': {period_ms},
{topics_block}                        'acc_window': {acc_window}
                    }},
                    {{
                        'qos_reliability': '{QOS_RELIABILITY}',
                        'qos_history': '{QOS_HISTORY}',
                        'qos_depth': {QOS_DEPTH},
                        'qos_durability': '{QOS_DURABILITY}',
                        'qos_deadline': {QOS_DEADLINE},
                        'qos_lifespan': {QOS_LIFESPAN}
                    }}
                ],
                extra_arguments=[{{'use_intra_process_comms': {USE_INTRAPROCESS}}}],
                {callback_group}),"""
        content.append(node_entry)

    # 封底
    if EXECUTOR_TYPE == "CallbackIsolatedExecutor":
        content.extend([
            "            ],",
            "            cie_config=cie_config",
            "        )",
            "    else:",
            "        # Fallback 到标准多线程容器",
            "        container = ComposableNodeContainer(",
            "            name='fins_eval_container',",
            "            namespace='',",
            "            package='rclcpp_components',",
            f"            executable='{executor_config['executable']}',",
            "            composable_node_descriptions=composable_nodes,",
            "            output='screen'",
            "        )",
            "    return launch.LaunchDescription([container])"
        ])
    else:
        content.extend([
            "        ],",
            "        output='screen',",
            f"        # Executor: {EXECUTOR_TYPE}, Threads: {executor_config['threads']}",
            "    )",
            "    return launch.LaunchDescription([container])"
        ])

    with open(launch_path, 'w') as f:
        f.write("\n".join(content))

def main():
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
        print(f"Created directory: {OUTPUT_DIR}")

    json_files = glob.glob(os.path.join(INPUT_DIR, "feedback_*.json"))

    if not json_files:
        print("No FINS JSON files found!")
        return

    print(f"Starting conversion of {len(json_files)} files...")
    print(f"Executor type: {EXECUTOR_TYPE}")
    print(f"Intra-process communication: {USE_INTRAPROCESS}")
    if EXECUTOR_TYPE == "CallbackIsolatedExecutor":
        print(f"CIE Priority: {CIE_THREAD_PRIORITY}")
        print(f"CIE Schedule Policy: {CIE_THREAD_SCHED_POLICY}")
        print(f"CIE CPU Affinity: {CIE_CPU_AFFINITY if CIE_CPU_AFFINITY else 'Not Set'}")
    print("-" * 60)

    for j_file in json_files:
        base_name = os.path.basename(j_file)
        launch_name = base_name.replace(".json", ".launch.py")
        launch_path = os.path.join(OUTPUT_DIR, launch_name)
        fins_json_to_component_launch(j_file, launch_path)
        print(f"  [Converted] {base_name} -> {launch_name}")

    print(f"\nDone! All launch files are in {OUTPUT_DIR}")

if __name__ == "__main__":
    main()