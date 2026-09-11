import argparse
import json
import os
import glob
import re

# ================= 配置区 =================
INPUT_DIR = "./result/CIE_FIFO_IPC_nuc12"
OUTPUT_DIR = "./result/CIE_FIFO_IPC_nuc12"
# USE_INTRAPROCESS: 是否启用 ROS 2 内部进程通信
USE_INTRAPROCESS = True

# ====== Executor 选择 ======
# 可选值:
#   - "MTE"   # MultiThreadedExecutor (使用 usr_container)
#   - "CIE"   # CallbackIsolatedExecutor (使用 cie_container)
EXECUTOR_TYPE = "CIE"  # 修改这里切换执行器

# ====== Executor 配置映射 ======
# 根据 EXECUTOR_TYPE 自动选择对应的包、组件和容器
EXECUTOR_CONFIG = {
    "MTE": {
        "package": "eval_mte",           # MTE 包名
        "component": "eval::MTEComponent",  # MTE 组件名（与 mte_component.cpp 注册一致）
        "executable": "mte_container",   # MTE 容器
        "note": "MultiThreadedExecutor (标准多线程执行器)"
    },
    "CIE": {
        "package": "eval_cie",           # CIE 包名
        "component": "eval::CIEComponent",  # CIE 组件名
        "executable": "cie_container",   # CIE 容器
        "note": "CallbackIsolatedExecutor (回调隔离执行器)"
    }
}

# 获取当前执行器配置
CURRENT_CONFIG = EXECUTOR_CONFIG[EXECUTOR_TYPE]
PACKAGE_NAME = CURRENT_CONFIG["package"]
COMPONENT_NAME = CURRENT_CONFIG["component"]
CONTAINER_EXECUTABLE = CURRENT_CONFIG["executable"]

# ====== CallbackIsolatedExecutor 专用配置 ======
CIE_THREAD_PRIORITY = 50
CIE_THREAD_SCHED_POLICY = "SCHED_OTHER"  # "SCHED_FIFO", "SCHED_RR", "SCHED_OTHER"
CIE_CPU_AFFINITY = [0, 1, 2]
# ===========================================

# ====== QoS 配置 ======
QOS_RELIABILITY = "reliable"
QOS_HISTORY = "keep_last"
QOS_DEPTH = 10
QOS_DURABILITY = "volatile"
QOS_DEADLINE = 0
QOS_LIFESPAN = 0
# ==========================

def apply_executor(executor_type):
    """按 executor 类型刷新派生的包/组件/容器常量（支持 CLI 切换）。"""
    global EXECUTOR_TYPE, CURRENT_CONFIG, PACKAGE_NAME, COMPONENT_NAME, \
        CONTAINER_EXECUTABLE
    EXECUTOR_TYPE = executor_type
    CURRENT_CONFIG = EXECUTOR_CONFIG[executor_type]
    PACKAGE_NAME = CURRENT_CONFIG["package"]
    COMPONENT_NAME = CURRENT_CONFIG["component"]
    CONTAINER_EXECUTABLE = CURRENT_CONFIG["executable"]


def parse_args():
    """命令行参数：一键切换 executor 与输入/输出目录。"""
    p = argparse.ArgumentParser(
        description="把 json 任务图生成 launch（默认 CIE，可切 MTE）")
    p.add_argument("--executor", choices=["CIE", "MTE"],
                   default=None, help="executor 类型（默认取模块常量 EXECUTOR_TYPE）")
    p.add_argument("--input", default=None, help="json 所在目录")
    p.add_argument("--output", default=None, help="launch 输出目录")
    return p.parse_args()


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
    threads = m_value if m_value > 0 else os.cpu_count()

    config = {
        "threads": threads,
        "executable": CONTAINER_EXECUTABLE,
        "note": f"{CURRENT_CONFIG['note']}，使用 {threads} 个线程 (m={m_value})",
        "is_cie": (EXECUTOR_TYPE == "CIE")
    }

    return config

def _fmt_number(value):
    """整数值按 int 输出"""
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
        f"# 包名: {PACKAGE_NAME}",
        f"# 组件: {COMPONENT_NAME}",
        f"# 可执行文件: {executor_config['executable']}",
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

    # 根据执行器类型构建容器参数
    if executor_config["is_cie"]:
        content.extend([
            "    # 使用 CallbackIsolatedExecutor (CIE)",
            f"    container = ComposableNodeContainer(",
            "        name='fins_eval_container',",
            "        namespace='',",
            f"        package='{PACKAGE_NAME}',",
            f"        executable='{executor_config['executable']}',",
            "        parameters=[{",
            f"            'num_threads': {executor_config['threads']},",
            f"            'thread_priority': {CIE_THREAD_PRIORITY},",
            f"            'scheduler_policy': '{CIE_THREAD_SCHED_POLICY}',",
            f"            'cpu_affinity': {CIE_CPU_AFFINITY if CIE_CPU_AFFINITY else []}",
            "        }],",
            "        composable_node_descriptions=[",
        ])
    else:
        content.extend([
            f"    # {CURRENT_CONFIG['note']}",
            "    container = ComposableNodeContainer(",
            "        name='fins_eval_container',",
            "        namespace='',",
            f"        package='{PACKAGE_NAME}',",
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

        _tind = "                        "
        topics_block = ""
        if inputs:
            topics_block += f"{_tind}'input_topics': {inputs!r},\n"
        if outputs:
            topics_block += f"{_tind}'output_topics': {outputs!r},\n"

        node_entry = f"""            ComposableNode(
                package='{PACKAGE_NAME}',
                plugin='{COMPONENT_NAME}',
                name='{node_id}',
                parameters=[
                    {{
                        'wcet_us': {wcet_us},
                        'period_ms': {period_ms},
{topics_block}                        'acc_window': {acc_window},
                        'node_id': '{node_id}'
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
            ),"""
        content.append(node_entry)

    # 封底
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
    args = parse_args()
    # CLI 覆盖模块常量（默认仍是配置区里的值）
    global INPUT_DIR, OUTPUT_DIR
    executor = args.executor if args.executor else EXECUTOR_TYPE
    input_dir = args.input if args.input else INPUT_DIR
    output_dir = args.output if args.output else OUTPUT_DIR
    INPUT_DIR, OUTPUT_DIR = input_dir, output_dir
    apply_executor(executor)

    # 验证执行器类型
    if EXECUTOR_TYPE not in EXECUTOR_CONFIG:
        print(f"[Error] 无效的执行器类型: {EXECUTOR_TYPE}")
        print(f"可选值: {list(EXECUTOR_CONFIG.keys())}")
        return

    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
        print(f"Created directory: {OUTPUT_DIR}")

    json_files = glob.glob(os.path.join(INPUT_DIR, "*.json"))

    if not json_files:
        print("No FINS JSON files found!")
        return

    print(f"Starting conversion of {len(json_files)} files...")
    print(f"Executor type: {EXECUTOR_TYPE}")
    print(f"Package: {PACKAGE_NAME}")
    print(f"Component: {COMPONENT_NAME}")
    print(f"Executable: {CONTAINER_EXECUTABLE}")
    print(f"Intra-process communication: {USE_INTRAPROCESS}")
    if EXECUTOR_TYPE == "CIE":
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