import os
import subprocess
import time
import shutil
import glob
from datetime import datetime

# ================= 配置区 =================
LAUNCH_DIR = "./launches"
RESULT_BASE_DIR = "./reports"
WARMUP_TIME = 5                # 增加到 5s，确保节点互联
RUN_TIME = 15.0                # 增加到 15s

# ROS 2 工作空间路径（修改为你的实际路径）
ROS2_WS_PATH = os.path.expanduser("~/Documents/GitHub/ros2_ws")  # 或你的工作空间路径
# ==========================================

def source_ros2_workspace():
    """
    Source ROS 2 工作空间，加载环境变量
    返回: True 表示成功，False 表示失败
    """
    setup_script = os.path.join(ROS2_WS_PATH, "install", "setup.bash")

    if not os.path.exists(setup_script):
        print(f"⚠️  警告: 找不到 setup.bash: {setup_script}")
        print("请检查 ROS2_WS_PATH 配置是否正确")
        return False

    print(f"📦 正在加载工作空间: {ROS2_WS_PATH}")

    # 执行 source 并获取环境变量
    cmd = f"bash -c 'source {setup_script} && env'"
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)

    if result.returncode != 0:
        print(f"❌ 错误: 无法 source 工作空间")
        return False

    # 解析并更新环境变量
    env_count = 0
    for line in result.stdout.strip().split('\n'):
        if '=' in line:
            key, value = line.split('=', 1)
            # 只更新关键的环境变量，避免覆盖 PATH 等导致问题
            if key in ['AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH',
                       'LD_LIBRARY_PATH', 'PYTHONPATH', 'ROS_DISTRO', 'ROS_VERSION']:
                os.environ[key] = value
                env_count += 1
            # 对于 PATH，需要特殊处理，追加而不是覆盖
            elif key == 'PATH':
                # 保留原有的 PATH，但将工作空间的 bin 目录添加到前面
                existing_path = os.environ.get('PATH', '')
                # 检查是否已经包含工作空间的 bin
                ws_bin = os.path.join(ROS2_WS_PATH, 'install', 'bin')
                if ws_bin not in existing_path:
                    os.environ['PATH'] = f"{ws_bin}:{existing_path}"
                    env_count += 1

    print(f"✅ 成功加载工作空间 (更新了 {env_count} 个环境变量)")
    return True

def check_eval_package():
    """
    检查 eval 包是否可用
    返回: True 表示可用，False 表示不可用
    """
    try:
        result = subprocess.run(["ros2", "pkg", "list"],
                                capture_output=True, text=True, timeout=5)
        if "eval" in result.stdout:
            print("✅ 找到 eval 包")
            return True
        else:
            print("❌ 未找到 eval 包")
            return False
    except Exception as e:
        print(f"❌ 检查包时出错: {e}")
        return False

def ensure_ros2_environment():
    """
    确保 ROS 2 环境已正确加载
    如果未加载，尝试自动加载
    返回: True 表示成功，False 表示失败
    """
    # 1. 首先检查当前环境是否有 eval 包
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
        print("❌ 环境加载后仍未找到 eval 包")
        print("请检查:")
        print("  1. eval 包是否已成功构建 (colcon build --packages-select eval)")
        print("  2. ROS2_WS_PATH 配置是否正确")
        return False

def run_single_test(launch_file):
    test_name = os.path.basename(launch_file).replace(".launch.py", "")
    timestamp = datetime.now().strftime("%H%M%S")
    session_name = f"eval_{test_name}"

    test_result_dir = os.path.join(RESULT_BASE_DIR, f"{test_name}_{timestamp}")
    os.makedirs(test_result_dir, exist_ok=True)

    print(f"\n>>> [开始测试] {test_name}")

    # 【修正 1】强制清理之前的 session，防止冲突
    subprocess.run(["ros2", "trace", "stop", session_name],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # 【修正 2】先开启追踪，再启动节点。
    #   必须在 ros2 launch 之前 start，否则组件加载瞬间的
    #   rcl_node_init / rclcpp_timer_link_node / callback_added 等
    #   “句柄→节点名”映射事件会落在录制窗口外，导出器将无法给出 tag。
    #   这些 init 事件只在加载时出现一次，量很小，不构成噪声。
    print(f"  [1/4] 开启追踪...")
    subprocess.run(["ros2", "trace", "start", session_name])

    # 【修正 3】不要用 PIPE，避免缓冲区满卡死；DEVNULL 即可
    launch_cmd = ["ros2", "launch", launch_file]
    print(f"  [2/4] 启动节点并预热 {WARMUP_TIME}s...")
    sim_proc = subprocess.Popen(launch_cmd,
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)

    time.sleep(WARMUP_TIME)

    # 2. 持续运行（仍在录制内）
    print(f"  [3/4] 持续运行 {RUN_TIME}s...")
    time.sleep(RUN_TIME)

    # 3. 停止追踪
    print(f"  [3/4] 停止追踪并导出数据...")
    trace_stop_cmd = ["ros2", "trace", "stop", session_name]
    subprocess.run(trace_stop_cmd)

    # 4. 关闭节点
    print(f"  [4/4] 关闭节点进程...")
    sim_proc.terminate()
    try:
        sim_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        sim_proc.kill()

    # 5. 搬运 Trace 数据
    default_trace_path = os.path.expanduser(f"~/.ros/tracing/{session_name}")
    final_trace_path = os.path.join(test_result_dir, "trace")

    if os.path.exists(default_trace_path):
        if os.path.exists(final_trace_path):
            shutil.rmtree(final_trace_path)
        shutil.move(default_trace_path, final_trace_path)
        print(f"  [成功] 数据已存至: {test_result_dir}")
    else:
        print(f"  [错误] 未找到数据。检查：1. 是否开启了 ros2_tracing 2. WARMUP_TIME 是否太短")

def main():
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