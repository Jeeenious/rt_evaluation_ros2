#!/usr/bin/env python3
# ==============================================================================
# _test.py - 自动运行 CIE/MTE 实时性评估实验控制脚本 (整理版)
# ==============================================================================
import argparse
import os
import re
import signal
import subprocess
import time
import shutil
import glob
from datetime import datetime

# ==============================================================================
# 1. 全局配置区
# ==============================================================================
LAUNCH_DIR = "./result/CIE_FIFO_IPC"
RESULT_BASE_DIR = "./result/CIE_FIFO_IPC"
WARMUP_TIME = 2.0  # 预热时间 (秒)
RUN_TIME = 100.0  # 持续运行时间 (秒)

# 断点续跑：默认跳过“结果目录下已有非空 trace”的用例（便于中断后接着跑）。
# 需要全部重跑时加 --rerun，或把此项改为 False。
SKIP_EXISTING_RESULTS = True

# 进程 CPU 绑定配置 (方案 A：taskset 整棵进程树)
# 核数根据 launch 文件名中的 m 自动解析（例如 feedback_..._m3_... 绑定 m 个核）
PIN_EXPERIMENT_TO_CPUS = True
CPU_OFFSET = 1  # 起始核号（避开核 0，通常留给系统内核与中断）

# ROS 2 工作空间路径
ROS2_WS_PATH = os.path.expanduser("~/ROSProjects/rt_eval_ws")

# 显式使能的自定义 UST tracepoint
CUSTOM_UST_EVENTS = [
    "eval:algo_execute",
    "eval:algo_complete",
    "eval:algo_working"
]

# 默认 ROS 2 追踪事件回退列表 (Jazzy / ros2_tracing DEFAULT_EVENTS_UST)
ROS2_DEFAULT_EVENTS_FALLBACK = [
    "ros2:rcl_init", "ros2:rcl_node_init", "ros2:rmw_publisher_init",
    "ros2:rcl_publisher_init", "ros2:rclcpp_publish", "ros2:rclcpp_intra_publish",
    "ros2:rcl_publish", "ros2:rmw_publish", "ros2:rmw_subscription_init",
    "ros2:rcl_subscription_init", "ros2:rclcpp_subscription_init",
    "ros2:rclcpp_subscription_callback_added", "ros2:rmw_take", "ros2:rcl_take",
    "ros2:rclcpp_take", "ros2:rcl_service_init", "ros2:rclcpp_service_callback_added",
    "ros2:rcl_client_init", "ros2:rcl_timer_init", "ros2:rclcpp_timer_callback_added",
    "ros2:rclcpp_timer_link_node", "ros2:rclcpp_callback_register",
    "ros2:callback_start", "ros2:callback_end",
    "ros2:rcl_lifecycle_state_machine_init", "ros2:rcl_lifecycle_transition",
    "ros2:rclcpp_executor_get_next_ready", "ros2:rclcpp_executor_wait_for_work",
    "ros2:rclcpp_executor_execute", "ros2:rclcpp_ipb_to_subscription",
    "ros2:rclcpp_buffer_to_ipb", "ros2:rclcpp_construct_ring_buffer",
    "ros2:rclcpp_ring_buffer_enqueue", "ros2:rclcpp_ring_buffer_dequeue",
    "ros2:rclcpp_ring_buffer_clear",
]

# 监控的 Linux 内核调度 tracepoint
KERNEL_EVENTS = [
    "sched_switch",
]


# ==============================================================================
# 2. 环境与辅助函数
# ==============================================================================
def get_default_ust_events():
    """优先从已安装的 tracetools_trace 读取默认 UST 事件表。"""
    try:
        code = "from tracetools_trace.tools import names;print(' '.join(names.DEFAULT_EVENTS_UST))"
        res = subprocess.run(["python3", "-c", code], capture_output=True, text=True, timeout=10)
        if res.returncode == 0 and res.stdout.strip():
            return res.stdout.split()
    except Exception as e:
        print(f"  ⚠️ 读取默认事件表失败 ({e})，回退到内置列表。")
    return list(ROS2_DEFAULT_EVENTS_FALLBACK)


def source_ros2_workspace():
    """加载 ROS 2 工作空间环境变量。"""
    setup_script = os.path.join(ROS2_WS_PATH, "install", "setup.bash")
    if not os.path.exists(setup_script):
        print(f"⚠️ 警告: 找不到 setup.bash: {setup_script}")
        return False

    print(f"📦 正在加载工作空间: {ROS2_WS_PATH}")
    cmd = f"bash -c 'source {setup_script} && env'"
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)

    if result.returncode != 0:
        print("❌ 错误: 无法 source 工作空间。")
        return False

    env_count = 0
    for line in result.stdout.strip().split('\n'):
        if '=' in line:
            key, value = line.split('=', 1)
            if key in ['AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH',
                       'LD_LIBRARY_PATH', 'PYTHONPATH', 'ROS_DISTRO', 'ROS_VERSION', 'PATH']:
                os.environ[key] = value
                env_count += 1

    print(f"✅ 成功加载工作空间 (更新了 {env_count} 个环境变量)。")
    return True


def check_eval_package():
    """检查 eval 相关功能包是否可用。"""
    try:
        result = subprocess.run(["ros2", "pkg", "list"], capture_output=True, text=True, timeout=5)
        return any(pkg in result.stdout for pkg in ["eval_cie", "eval_mte", "eval"])
    except Exception:
        return False


def ensure_ros2_environment():
    """确保 ROS 2 环境及评测包已正确加载。"""
    if check_eval_package():
        print("✅ 找到 eval 系列功能包。")
        return True

    print("🔄 环境未包含 eval 包，尝试加载工作空间...")
    if not source_ros2_workspace():
        return False

    if check_eval_package():
        print("✅ 环境加载成功。")
        return True

    print("❌ 环境加载后仍未找到 eval 包，请确认 colcon build 已成功运行。")
    return False


def parse_m_from_name(name):
    """从文件名中解析并发度 m 值。"""
    for pattern in [r'_m(\d+)', r'm(\d+)_']:
        m = re.search(pattern, name)
        if m:
            return int(m.group(1))
    return 0


def find_completed_result(test_name):
    """该用例是否已有完成的结果；有则返回其 trace 目录，否则 None。

    结果目录命名：<RESULT_BASE_DIR>/<test_name>_<HHMMSS>/trace
    trace 只在实验成功归档后才出现，故“目录存在且非空”即视为已完成。
    """
    pattern = os.path.join(RESULT_BASE_DIR, f"{test_name}_*", "trace")
    for trace_dir in sorted(glob.glob(pattern)):
        if os.path.isdir(trace_dir) and os.listdir(trace_dir):
            return trace_dir
    return None


# ==============================================================================
# 3. 单个实验执行逻辑
# ==============================================================================
def run_single_test(launch_file):
    test_name = os.path.basename(launch_file).replace(".launch.py", "")
    timestamp = datetime.now().strftime("%H%M%S")
    session_name = f"eval_{test_name}_{timestamp}"
    test_result_dir = os.path.join(RESULT_BASE_DIR, f"{test_name}_{timestamp}")
    os.makedirs(test_result_dir, exist_ok=True)

    print(f"\n>>> [开始测试] {test_name}")

    # 1. 清理残存同名 Session
    subprocess.run(["ros2", "trace", "stop", session_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    default_trace_path = os.path.expanduser(f"~/.ros/tracing/{session_name}")
    if os.path.exists(default_trace_path):
        shutil.rmtree(default_trace_path)

    # 2. 准备子进程环境变量 (注入 LD_PRELOAD 库与注册超时)
    child_env = os.environ.copy()
    lttng_ust_paths = [
        "/usr/lib/x86_64-linux-gnu/liblttng-ust.so",
        "/usr/lib/x86_64-linux-gnu/liblttng-ust.so.1",
        "/usr/lib64/liblttng-ust.so"
    ]
    lttng_ust_path = next((p for p in lttng_ust_paths if os.path.exists(p)), None)

    if lttng_ust_path:
        existing_preload = child_env.get("LD_PRELOAD", "")
        child_env["LD_PRELOAD"] = f"{lttng_ust_path}:{existing_preload}" if existing_preload else lttng_ust_path
        print(f"  [环境] 注入 LTTng UST 库: {lttng_ust_path}")
    else:
        child_env["LD_PRELOAD"] = "liblttng-ust.so"

    child_env["LTTNG_UST_REGISTER_TIMEOUT"] = "-1"

    # 3. 启动 Trace 追踪 (合并默认 ros2 事件与自定义 eval 事件)
    print(f"  [1/5] 开启 Trace 追踪...")
    ust_events = get_default_ust_events() + CUSTOM_UST_EVENTS
    trace_start_cmd = ["ros2", "trace", "start", session_name, "-u"] + ust_events + ["-k"] + KERNEL_EVENTS

    trace_start_res = subprocess.run(trace_start_cmd, capture_output=True, text=True)
    if trace_start_res.returncode != 0:
        print(f"  ❌ ros2 trace 启动失败:\n{trace_start_res.stderr}")
        return
    print(f"  ✅ Trace Session 创建成功。")
    time.sleep(1.5)

    # 4. 启动 C++ 节点进程 (支持 taskset 绑定核)
    launch_cmd = ["ros2", "launch", launch_file]
    if PIN_EXPERIMENT_TO_CPUS:
        m = parse_m_from_name(test_name)
        n_cores = m if m > 0 else os.cpu_count() or 1
        cores = ",".join(str(CPU_OFFSET + i) for i in range(n_cores))
        launch_cmd = ["taskset", "-c", cores] + launch_cmd
        print(f"  [2/5] 启动 C++ 进程 (taskset -c {cores}, m={m})...")
    else:
        print(f"  [2/5] 启动 C++ 进程...")

    sim_proc = subprocess.Popen(
        launch_cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        env=child_env,
        start_new_session=True  # 独立进程组，便于事后整体清理
    )

    # 5. 预热与录制
    print(f"  [3/5] 预热 {WARMUP_TIME}s 并持续录制 {RUN_TIME}s...")
    time.sleep(WARMUP_TIME + RUN_TIME)

    # 6. 停止 Trace
    print(f"  [4/5] 停止 Trace 追踪...")
    subprocess.run(["ros2", "trace", "stop", session_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # 7. 清理实验进程组 (killpg)
    print(f"  [5/5] 关闭并清理实验进程 (pgid={sim_proc.pid})...")
    try:
        os.killpg(sim_proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass

    try:
        sim_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(sim_proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        sim_proc.wait()

    # 8. 搬运与归档 Trace 数据
    final_trace_path = os.path.join(test_result_dir, "trace")
    if os.path.exists(default_trace_path):
        if os.path.exists(final_trace_path):
            shutil.rmtree(final_trace_path)
        shutil.move(default_trace_path, final_trace_path)
        print(f"  [成功] Trace 数据已归档: {test_result_dir}")

        # 9. babeltrace2 校验数据流
        bt_cmd = f"babeltrace2 '{final_trace_path}'"
        res = subprocess.run(bt_cmd, shell=True, capture_output=True, text=True)
        eval_lines = [line for line in res.stdout.splitlines() if "eval:" in line]

        if len(eval_lines) > 0:
            print(f"  [验证] ✅ 成功捕捉到 {len(eval_lines)} 条 eval 自定义事件！")
        elif len(res.stdout.strip()) > 0:
            print(f"  [验证] ⚠️ 仅捕获到 ROS 2 默认事件，未捕捉到 eval 自定义事件 (0条)。")
        else:
            print(f"  [验证] ❌ Trace 目录为空，未录制到任何事件。")
    else:
        print(f"  [错误] 未在 ~/.ros/tracing/ 找到录制结果。")


# ==============================================================================
# 4. 主入口
# ==============================================================================
def parse_args():
    p = argparse.ArgumentParser(description="自动化运行 CIE/MTE 实验控制脚本")
    p.add_argument("--launch-dir", default=None, help="launch 文件所在目录")
    p.add_argument("--result-dir", default=None, help="结果输出目录")
    p.add_argument("--rerun", action="store_true",
                   help="强制重跑已有结果的用例（默认跳过已有非空 trace 的用例）")
    return p.parse_args()


def main():
    global LAUNCH_DIR, RESULT_BASE_DIR
    args = parse_args()
    if args.launch_dir:
        LAUNCH_DIR = args.launch_dir
    if args.result_dir:
        RESULT_BASE_DIR = args.result_dir

    print("=" * 60)
    print("🔧 初始化 ROS 2 实时测试环境...")
    print("=" * 60)
    print(f"🔧 目录配置: LAUNCH_DIR={LAUNCH_DIR} | RESULT_BASE_DIR={RESULT_BASE_DIR}")

    if not ensure_ros2_environment():
        print("\n❌ 环境初始化失败，程序退出。")
        return

    if not os.path.exists(LAUNCH_DIR):
        print(f"❌ 错误: 找不到 launch 目录 {LAUNCH_DIR}")
        return

    os.makedirs(RESULT_BASE_DIR, exist_ok=True)
    launch_files = sorted(glob.glob(os.path.join(LAUNCH_DIR, "*.launch.py")))
    print(f"\n📁 发现 {len(launch_files)} 个待测实验用例。")

    skip_existing = SKIP_EXISTING_RESULTS and not args.rerun
    print(f"🔁 断点续跑: {'开启（跳过已有结果）' if skip_existing else '关闭（全部重跑）'}")

    skipped = 0
    for i, l_file in enumerate(launch_files):
        test_name = os.path.basename(l_file).replace(".launch.py", "")

        if skip_existing:
            done_trace = find_completed_result(test_name)
            if done_trace:
                skipped += 1
                print(f"\n⏭️  跳过（已有结果）: {test_name}")
                print(f"     {os.path.relpath(done_trace, RESULT_BASE_DIR)}")
                continue

        print(f"\n----------------------------------------")
        print(f"📊 进度: [{i + 1}/{len(launch_files)}] -> {os.path.basename(l_file)}")
        print(f"----------------------------------------")
        run_single_test(l_file)
        time.sleep(2)

    if skipped:
        print(f"\n⏭️  共跳过 {skipped} 个已有结果的用例（加 --rerun 可强制重跑）。")
    print("\n✨ 所有实验测试已顺利完成！")


if __name__ == "__main__":
    main()