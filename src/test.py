import argparse
import os
import re
import signal
import subprocess
import time
import shutil
import glob
from datetime import datetime

# ================= 配置区 =================
LAUNCH_DIR = "./result/CIE_FIFO_IPC_nuc12"
RESULT_BASE_DIR = "./result/CIE_FIFO_IPC_nuc12"
WARMUP_TIME = 5                # 预热时间 (s)
RUN_TIME = 5.0                 # 持续运行时间 (s)

# ===== 是否把实验进程钉到 CPU 上（方案 A：taskset 整棵进程树）=====
# 核数 = json/launch 文件名中解析出的 m（如 feedback_..._m3_... → 钉 m 个核）
PIN_EXPERIMENT_TO_CPUS = True
CPU_OFFSET = 1                # 起始核号：绕开 0，从 1 开始（0 常用于内核/中断）

# ==========================================

# ROS 2 工作空间路径
ROS2_WS_PATH = os.path.expanduser("~/ROSProjects/rt_eval_ws")
# ==========================================

# ===== 需要显式使能的自定义 UST tracepoint =====
# 关键：`ros2 trace start` 在不传 `-u` 时，只会启用内置的默认事件表
# (tracetools_trace.tools.names.DEFAULT_EVENTS_UST，即 35 个 ros2:* 事件)；
# 而 `-a` 选项 = "--append-trace"（目录已存在时允许追加），**不是**“启用所有事件”。
# 因此自定义事件 (eval:algo_*) 必须通过 `-u` 显式列出，否则无论探针是否触发都不会被录到。
CUSTOM_UST_EVENTS = ["eval:algo_execute", "eval:algo_complete", "eval:algo_working"]

# 兜底：若运行期无法从 tracetools_trace 读取默认事件表，则回退到这份硬编码列表
# (Jazzy / ros2_tracing 的 DEFAULT_EVENTS_UST，按需增删)
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

# ===== 需要监控的 Linux scheduler kernel tracepoint =====
KERNEL_EVENTS = [
    "sched_switch",
    # "sched_wakeup",
    # "sched_migrate_task",
]

def get_default_ust_events():
    """优先从已安装的 tracetools_trace 读取默认 UST 事件表（随发行版同步）。"""
    try:
        # 继承当前环境（ensure_ros2_environment 已把 PYTHONPATH 指到 ROS 发行版）
        code = "from tracetools_trace.tools import names;print(' '.join(names.DEFAULT_EVENTS_UST))"
        res = subprocess.run(["python3", "-c", code],
                             capture_output=True, text=True, timeout=10)
        if res.returncode == 0 and res.stdout.strip():
            return res.stdout.split()
    except Exception as e:
        print(f"  ⚠️ 读取默认事件表失败({e})，回退到内置列表")
    return list(ROS2_DEFAULT_EVENTS_FALLBACK)
# ==========================================

def source_ros2_workspace():
    """Source ROS 2 工作空间，加载环境变量"""
    setup_script = os.path.join(ROS2_WS_PATH, "install", "setup.bash")

    if not os.path.exists(setup_script):
        print(f"⚠️  警告: 找不到 setup.bash: {setup_script}")
        return False

    print(f"📦 正在加载工作空间: {ROS2_WS_PATH}")

    # 执行 source 并获取环境变量
    cmd = f"bash -c 'source {setup_script} && env'"
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)

    if result.returncode != 0:
        print("❌ 错误: 无法 source 工作空间")
        return False

    # 解析并更新环境变量
    env_count = 0
    for line in result.stdout.strip().split('\n'):
        if '=' in line:
            key, value = line.split('=', 1)
            # 只更新关键的环境变量，避免覆盖其它无关变量导致问题
            if key in ['AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH',
                       'LD_LIBRARY_PATH', 'PYTHONPATH', 'ROS_DISTRO', 'ROS_VERSION',
                       'PATH']:
                # PATH 直接采用 source 后的完整值（已包含 /opt/ros/<distro>/bin、
                # 工作空间 install/bin 及原有系统路径），否则找不到 ros2 等命令
                os.environ[key] = value
                env_count += 1

    print(f"✅ 成功加载工作空间 (更新了 {env_count} 个环境变量)")
    return True

def check_eval_package():
    """检查 eval 相关包是否可用"""
    try:
        result = subprocess.run(["ros2", "pkg", "list"],
                                capture_output=True, text=True, timeout=5)
        if "eval_cie" in result.stdout or "eval_mte" in result.stdout or "eval" in result.stdout:
            print("✅ 找到 eval 系列功能包")
            return True
        else:
            print("❌ 未找到 eval 功能包")
            return False
    except Exception as e:
        print(f"❌ 检查包时出错: {e}")
        return False

def ensure_ros2_environment():
    """确保 ROS 2 环境已正确加载"""
    if check_eval_package():
        return True

    # 2. 如果没有，尝试 source 工作空间
    print("🔄 环境未包含 eval 包，尝试加载工作空间...")
    if not source_ros2_workspace():
        return False

    # 3. 再次检查是否成功
    if check_eval_package():
        print("✅ 环境加载成功")
        return True
    else:
        print("❌ 环境加载后仍未找到 eval 包，请确认 colcon build 已成功运行")
        return False

def parse_m_from_name(name):
    """按 json/launch 文件名解析 m 值（与 json2launch.py 保持一致）。"""
    m = re.search(r'_m(\d+)', name)
    if m:
        return int(m.group(1))
    m = re.search(r'm(\d+)_', name)
    if m:
        return int(m.group(1))
    return 0

def run_single_test(launch_file):
    test_name = os.path.basename(launch_file).replace(".launch.py", "")
    timestamp = datetime.now().strftime("%H%M%S")
    session_name = f"eval_{test_name}_{timestamp}"

    test_result_dir = os.path.join(RESULT_BASE_DIR, f"{test_name}_{timestamp}")
    os.makedirs(test_result_dir, exist_ok=True)

    print(f"\n>>> [开始测试] {test_name}")

    # 1. 清理残存的同名 session 及其输出目录（避免残留目录导致 start 失败或旧数据被追加）
    subprocess.run(["ros2", "trace", "stop", session_name],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    default_trace_path = os.path.expanduser(f"~/.ros/tracing/{session_name}")
    if os.path.exists(default_trace_path):
        shutil.rmtree(default_trace_path)

    # 2. 准备环境变量（注入 LD_PRELOAD + UST 超时控制）
    child_env = os.environ.copy()
    possible_paths = [
        "/usr/lib/x86_64-linux-gnu/liblttng-ust.so",
        "/usr/lib/x86_64-linux-gnu/liblttng-ust.so.1",
        "/usr/lib64/liblttng-ust.so"
    ]

    lttng_ust_path = None
    for p in possible_paths:
        if os.path.exists(p):
            lttng_ust_path = p
            break

    if lttng_ust_path:
        existing_preload = child_env.get("LD_PRELOAD", "")
        child_env["LD_PRELOAD"] = f"{lttng_ust_path}:{existing_preload}" if existing_preload else lttng_ust_path
        print(f"  [环境] 注入 LTTng UST 库: {lttng_ust_path}")
    else:
        child_env["LD_PRELOAD"] = "liblttng-ust.so"

    # 防止节点启动时与 daemon 建立连接失败
    child_env["LTTNG_UST_REGISTER_TIMEOUT"] = "-1"

    # 3. 先开启 Trace 追踪（开启 Session 监听）
    #    显式使能 = 默认 ros2:* 事件 ∪ 自定义 eval:* 事件。
    #    ⚠️ 注意：`-u` 会【替换】默认事件表而不是追加，因此两者必须一起列出；
    #    仅写 `-u eval:*` 会丢掉 ros2:* 事件，导致 lttng2csv.py 无法解析框架事件。
    print(f"  [1/5] 开启 Trace 追踪...")
    ust_events = get_default_ust_events() + CUSTOM_UST_EVENTS
    trace_start_cmd = [
        "ros2", "trace", "start",
        session_name ] + [
        "-u",
    ] + ust_events + [
        "-k",
    ] + KERNEL_EVENTS

    trace_start_res = subprocess.run(trace_start_cmd, capture_output=True, text=True)
    if trace_start_res.returncode != 0:
        print(f"  ❌ ros2 trace 启动失败:\n{trace_start_res.stderr}")
    else:
        print(f"  ✅ Trace Session 创建成功")
    time.sleep(1.5)

    # 4. 再启动 C++ 节点进程
    #    方案 A：可选按文件名 m 值决定核数，用 taskset 把整棵进程树钉到这些核上
    #    （任务 m 个线程 → CPU_OFFSET..CPU_OFFSET+m-1 个核）
    launch_cmd = ["ros2", "launch", launch_file]
    if PIN_EXPERIMENT_TO_CPUS:
        m = parse_m_from_name(test_name)
        n_cores = m if m > 0 else os.cpu_count() or 1
        cores = ",".join(str(CPU_OFFSET + i) for i in range(n_cores))
        launch_cmd = ["taskset", "-c", cores] + launch_cmd
        print(f"  [2/5] 启动 C++ 节点进程 (taskset -c {cores}, m={m})...")
    else:
        print(f"  [2/5] 启动 C++ 节点进程...")
    # start_new_session：让整个实验进程独立成组，结束时可整组清理，杜绝容器残留
    sim_proc = subprocess.Popen(
        launch_cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        env=child_env,
        start_new_session=True
    )

    # 5. 持续运行与预热
    print(f"  [3/5] 预热 {WARMUP_TIME}s 并持续录制 {RUN_TIME}s...")
    time.sleep(WARMUP_TIME + RUN_TIME)

    # 6. 停止 Trace 追踪
    print(f"  [4/5] 停止 Trace 追踪...")
    subprocess.run(["ros2", "trace", "stop", session_name],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # 7. 关闭并清理本次实验进程（进程组整组清理）
    #    实验以独立进程组启动（start_new_session），这里 killpg 一次性杀掉
    #    ros2 launch + 容器(cie_container/mte_container) 及其子孙进程，
    #    避免残留容器把上轮负载录进下一轮 trace（进程级 UST 全局录制）。
    print(f"  [5/5] 关闭并清理实验进程 (pgid={sim_proc.pid})...")
    try:
        os.killpg(sim_proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass  # 进程组已不存在（launch 提前退出）
    try:
        sim_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(sim_proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            sim_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            sim_proc.kill()
            sim_proc.wait()

    # 8. 搬运 Trace 数据
    default_trace_path = os.path.expanduser(f"~/.ros/tracing/{session_name}")
    final_trace_path = os.path.join(test_result_dir, "trace")

    if os.path.exists(default_trace_path):
        if os.path.exists(final_trace_path):
            shutil.rmtree(final_trace_path)
        shutil.move(default_trace_path, final_trace_path)
        print(f"  [成功] Trace 数据存储路径: {test_result_dir}")

        # 9. 精确的数据流校验
        bt_cmd = f"babeltrace2 '{final_trace_path}'"
        res = subprocess.run(bt_cmd, shell=True, capture_output=True, text=True)
        eval_lines = [line for line in res.stdout.splitlines() if "eval:" in line]

        if len(eval_lines) > 0:
            print(f"  [验证] ✅ 成功捕捉到 {len(eval_lines)} 条 eval 自定义事件数据！")
        elif len(res.stdout.strip()) > 0:
            print(f"  [验证] ⚠️ 只捕获到了 ROS 2 框架默认事件，未能触发 eval 自定义事件 (0条)。")
        else:
            print(f"  [验证] ❌ Trace 目录为空，未录制到任何事件。")
    else:
        print(f"  [错误] 未在 ~/.ros/tracing/ 找到录制结果。")

def parse_args():
    """一键切换：--executor CIE|MTE 即指向预设目录，也可用 --launch-dir/--result-dir 覆盖。"""
    p = argparse.ArgumentParser(description="自动跑 CIE/MTE 实验（默认 CIE，带 taskset 钉核）")
    p.add_argument("--executor", choices=["CIE", "MTE"], default=None,
                   help="用预设目录一键切换（CIE/MTE）")
    p.add_argument("--launch-dir", default=None, help="launch 文件所在目录（覆盖预设）")
    p.add_argument("--result-dir", default=None, help="结果输出目录（覆盖预设）")
    return p.parse_args()


def main():
    global LAUNCH_DIR, RESULT_BASE_DIR
    args = parse_args()
    if args.launch_dir:
        LAUNCH_DIR = args.launch_dir
    if args.result_dir:
        RESULT_BASE_DIR = args.result_dir
    print(f"🔧 实验目录: LAUNCH_DIR={LAUNCH_DIR}  RESULT_BASE_DIR={RESULT_BASE_DIR}")

    # ===== 新增：确保 ROS 2 环境已加载 =====
    print("=" * 60)
    print("🔧 初始化 ROS 2 环境...")
    print("=" * 60)

    if not ensure_ros2_environment():
        print("\n❌ 环境初始化失败，退出程序")
        return

    print("=" * 60)
    print("✅ 环境准备完成，开始测试")
    print("=" * 60)
    # ======================================

    if not os.path.exists(LAUNCH_DIR):
        print(f"错误: 找不到 launch 目录 {LAUNCH_DIR}")
        return

    if not os.path.exists(RESULT_BASE_DIR):
        os.makedirs(RESULT_BASE_DIR)

    launch_files = sorted(glob.glob(os.path.join(LAUNCH_DIR, "*.launch.py")))
    print(f"\n发现 {len(launch_files)} 个案例。")

    for i, l_file in enumerate(launch_files):
        print(f"\n进度: {i+1}/{len(launch_files)}")
        run_single_test(l_file)
        time.sleep(2)

    print("\n✅ 所有测试完成。")

if __name__ == "__main__":
    main()