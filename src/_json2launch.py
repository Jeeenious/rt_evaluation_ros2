# ==============================================================================
# json2launch.py - 将 FINS JSON 任务图转为 ROS 2 Launch 文件
# ==============================================================================
import os
import glob
import re
import json
import argparse

# ==========================================
# ⚙️ 在这里修改你想使用的实验配置 (可选填 8 种之一)
# 可选值:
#   - CIE_FIFO_IPC, CIE_FIFO_noIPC
#   - CIE_RR_IPC, CIE_RR_noIPC
#   - CIE_OTHER_IPC, CIE_OTHER_noIPC
#   - MTE_IPC, MTE_noIPC
# ==========================================
SELECTED_CONFIG = "CIE_FIFO_IPC"

# 默认路径配置
DEFAULT_INPUT_DIR = "./result/ROS"
DEFAULT_OUTPUT_DIR = "./result/ROS"

# 8 种实验配置字典定义
EXPERIMENT_CONFIGS = {
    "CIE_FIFO_IPC": {
        "executor_type": "CIE",
        "package": "eval_cie",
        "component": "eval::CIEComponent",
        "executable": "cie_container",
        "sched_policy": "SCHED_FIFO",
        "thread_priority": 50,
        "use_intra_process": True,
        "note": "CallbackIsolatedExecutor, SCHED_FIFO, IPC Enabled"
    },
    "CIE_FIFO_noIPC": {
        "executor_type": "CIE",
        "package": "eval_cie",
        "component": "eval::CIEComponent",
        "executable": "cie_container",
        "sched_policy": "SCHED_FIFO",
        "thread_priority": 50,
        "use_intra_process": False,
        "note": "CallbackIsolatedExecutor, SCHED_FIFO, IPC Disabled"
    },
    "CIE_RR_IPC": {
        "executor_type": "CIE",
        "package": "eval_cie",
        "component": "eval::CIEComponent",
        "executable": "cie_container",
        "sched_policy": "SCHED_RR",
        "thread_priority": 50,
        "use_intra_process": True,
        "note": "CallbackIsolatedExecutor, SCHED_RR, IPC Enabled"
    },
    "CIE_RR_noIPC": {
        "executor_type": "CIE",
        "package": "eval_cie",
        "component": "eval::CIEComponent",
        "executable": "cie_container",
        "sched_policy": "SCHED_RR",
        "thread_priority": 50,
        "use_intra_process": False,
        "note": "CallbackIsolatedExecutor, SCHED_RR, IPC Disabled"
    },
    "CIE_OTHER_IPC": {
        "executor_type": "CIE",
        "package": "eval_cie",
        "component": "eval::CIEComponent",
        "executable": "cie_container",
        "sched_policy": "SCHED_OTHER",
        "thread_priority": 0,
        "use_intra_process": True,
        "note": "CallbackIsolatedExecutor, SCHED_OTHER, IPC Enabled"
    },
    "CIE_OTHER_noIPC": {
        "executor_type": "CIE",
        "package": "eval_cie",
        "component": "eval::CIEComponent",
        "executable": "cie_container",
        "sched_policy": "SCHED_OTHER",
        "thread_priority": 0,
        "use_intra_process": False,
        "note": "CallbackIsolatedExecutor, SCHED_OTHER, IPC Disabled"
    },
    "MTE_IPC": {
        "executor_type": "MTE",
        "package": "eval_mte",
        "component": "eval::MTEComponent",
        "executable": "mte_container",
        "sched_policy": "SCHED_FIFO", # 无效参数
        "thread_priority": 50,
        "use_intra_process": True,
        "note": "MultiThreadedExecutor, IPC Enabled"
    },
    "MTE_noIPC": {
        "executor_type": "MTE",
        "package": "eval_mte",
        "component": "eval::MTEComponent",
        "executable": "mte_container",
        "sched_policy": "SCHED_FIFO",  # 无效参数
        "thread_priority": 50,
        "use_intra_process": False,
        "note": "MultiThreadedExecutor, IPC Disabled"
    }
}

QOS_RELIABILITY = "reliable"
QOS_HISTORY = "keep_last"
QOS_DEPTH = 10
QOS_DURABILITY = "volatile"
QOS_DEADLINE = 0
QOS_LIFESPAN = 0


def parse_m_from_filename(filename):
    match = re.search(r'_m(\d+)', filename)
    if match: return int(match.group(1))
    match = re.search(r'm(\d+)_', filename)
    if match: return int(match.group(1))
    return 1


def _fmt_number(value):
    if isinstance(value, float) and value.is_integer():
        return int(value)
    return value


def fins_json_to_component_launch(json_path, launch_path, config_key):
    cfg = EXPERIMENT_CONFIGS[config_key]
    pkg_name = cfg["package"]
    comp_name = cfg["component"]
    container_exec = cfg["executable"]
    executor_type = cfg["executor_type"]
    sched_policy = cfg["sched_policy"]
    thread_priority = cfg["thread_priority"]
    use_intra_process = cfg["use_intra_process"]

    try:
        with open(json_path, 'r') as f:
            pipeline = json.load(f)
    except Exception as e:
        print(f"  [Error] Failed to parse {json_path}: {e}")
        return False

    filename = os.path.basename(json_path)
    m_value = parse_m_from_filename(filename)
    threads = m_value if m_value > 0 else os.cpu_count()
    is_cie = (executor_type == "CIE")

    # 动态根据文件名中的 m 值决定 cpu_affinity (例如 m3 -> [0, 1, 2])
    cpu_affinity = list(range(m_value)) if m_value > 0 else [0, 1, 2]

    content = [
        "import launch",
        "from launch_ros.actions import ComposableNodeContainer",
        "from launch_ros.descriptions import ComposableNode",
        "",
        "def generate_launch_description():",
    ]

    if is_cie:
        content.extend([
            f"    container = ComposableNodeContainer(",
            "        name='fins_eval_container',",
            "        namespace='',",
            f"        package='{pkg_name}',",
            f"        executable='{container_exec}',",
            "        parameters=[{",
            f"            'num_threads': {threads},",
            f"            'thread_priority': {thread_priority},",
            f"            'scheduler_policy': '{sched_policy}',",
            f"            'cpu_affinity': {cpu_affinity}",
            "        }],",
            "        composable_node_descriptions=[",
        ])
    else:
        content.extend([
            f"    container = ComposableNodeContainer(",
            "        name='fins_eval_container',",
            "        namespace='',",
            f"        package='{pkg_name}',",
            f"        executable='{container_exec}',",
            f"        parameters=[{{'num_threads': {threads}}}],",
            "        composable_node_descriptions=[",
        ])

    for node_cfg in pipeline:
        node_id = node_cfg.get("id", "unknown")
        wcet_us = 0
        if "parameters" in node_cfg and len(node_cfg["parameters"]) > 0:
            wcet_us = _fmt_number(node_cfg["parameters"][0].get("value", 0))

        period_ms = _fmt_number(node_cfg.get("period", 0))
        inputs = node_cfg.get("inputs", [])
        outputs = node_cfg.get("outputs", [])
        acc_window = list(node_cfg["loop"].values())[0] if "loop" in node_cfg and node_cfg["loop"] else 0

        topics_block = ""
        if inputs:
            topics_block += f"                        'input_topics': {inputs!r},\n"
        if outputs:
            topics_block += f"                        'output_topics': {outputs!r},\n"

        node_entry = f"""            ComposableNode(
                package='{pkg_name}',
                plugin='{comp_name}',
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
                extra_arguments=[{{'use_intra_process_comms': {use_intra_process}}}],
            ),"""
        content.append(node_entry)

    content.extend([
        "        ],",
        "        output='screen'",
        "    )",
        "    return launch.LaunchDescription([container])"
    ])

    os.makedirs(os.path.dirname(launch_path), exist_ok=True)
    with open(launch_path, 'w') as f:
        f.write("\n".join(content))
    return True


def main():
    parser = argparse.ArgumentParser(description="生成指定单配置的 Launch 文件")
    parser.add_argument("--input", default=DEFAULT_INPUT_DIR, help="JSON 输入目录")
    parser.add_argument("--output", default=DEFAULT_OUTPUT_DIR, help="输出目录 LAUNCH")
    parser.add_argument("--config", default=SELECTED_CONFIG, choices=list(EXPERIMENT_CONFIGS.keys()),
                        help="选择要生成的配置名称")
    args = parser.parse_args()

    config_key = args.config
    output_dir = args.output if args.output else os.path.join("./result", config_key)

    json_files = glob.glob(os.path.join(args.input, "*.json"))
    if not json_files:
        print(f"❌ 未在 {args.input} 下找到任何 FINS JSON 文件！")
        return

    os.makedirs(output_dir, exist_ok=True)
    print(f"\n[Generator] 正在生成单配置: {config_key} ({EXPERIMENT_CONFIGS[config_key]['note']})")
    print(f"📁 输入目录: {args.input}")
    print(f"📁 输出目录: {output_dir}")

    count = 0
    for j_file in json_files:
        base_name = os.path.basename(j_file)
        launch_name = base_name.replace(".json", ".launch.py")
        launch_path = os.path.join(output_dir, launch_name)
        if fins_json_to_component_launch(j_file, launch_path, config_key=config_key):
            m_val = parse_m_from_filename(base_name)
            print(f"  [Converted] {base_name} (m={m_val}) -> {launch_name}")
            count += 1

    print(f"✨ 成功生成 {count} 个 Launch 文件！\n")


if __name__ == "__main__":
    main()