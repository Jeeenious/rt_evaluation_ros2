# LTTng 安装与使用

本实验环境：

- Ubuntu 24.04

- Linux `7.0.0-31-generic`

- ROS 2 Jazzy

- LTTng-modules `2.14.6`

用于同时采集 ROS 2 userspace trace 和 Linux scheduler trace。

## 1. 安装 LTTng-modules

检查当前 kernel：

```bash
uname -r
```

安装编译依赖：

```bash
sudo apt update
sudo apt install build-essential linux-headers-$(uname -r) libelf-dev libdw-dev
```

下载并编译 LTTng-modules 2.14.6：

```bash
cd ~/Downloads

wget https://lttng.org/files/lttng-modules/lttng-modules-2.14.6.tar.bz2

tar -xjf lttng-modules-2.14.6.tar.bz2

cd lttng-modules-2.14.6

make
```

安装：

```bash
sudo make modules_install
sudo depmod -a
```

加载：

```bash
sudo modprobe lttng-tracer
```

检查：

```bash
lsmod | grep lttng
```

## 2. 验证 Scheduler Trace

检查 LTTng kernel events：

```bash
sudo lttng list --kernel
```

确认存在：

```text
sched_waking
sched_wakeup
sched_switch
```

本实验主要使用这三个事件：

```text
sched_waking
sched_wakeup
sched_switch
```

其中 `sched_switch` 用于确定 CPU 从哪个 thread 切换到哪个 thread。

## 3. ROS 2 Trace

加载 ROS 2：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ROSProjects/rt_eval_ws/install/setup.bash
```

启动 userspace + kernel tracing：

```bash
ros2 trace start <session_name> \
    -u \
    -k sched_waking \
    -k sched_wakeup \
    -k sched_switch
```

实验结束：

```bash
ros2 trace stop
```

查看 trace：

```bash
babeltrace2 ~/.ros/tracing/session
```

查看 scheduler events：

```bash
babeltrace2 ~/.ros/tracing/session \
    | grep -E 'sched_(waking|wakeup|switch)'
```

查看 ROS callback events：

```bash
babeltrace2 ~/.ros/tracing/session \
    | grep -E 'callback_(start|end)'
```

如果使用自定义 `eval` events：

```bash
babeltrace2 ~/.ros/tracing/session \
    | grep -E 'eval:(algo_execute|algo_complete|algo_working)'
```

## 4. Python 实验配置

Python 脚本中：

```python
KERNEL_EVENTS = [
    "sched_waking",
    "sched_wakeup",
    "sched_switch",
]
```

启动 tracing：

```python
trace_start_cmd = [
    "ros2",
    "trace",
    "start",
    session_name,
] + ["-u"] + ust_events + ["-k"] + KERNEL_EVENTS
```

不再使用 ftrace，直接使用 LTTng kernel tracer。

最终一次实验同时得到：

```text
ROS 2 UST
    callback_start
    callback_end
    executor events
    eval:* events

Linux kernel
    sched_waking
    sched_wakeup
    sched_switch
```

用于分析：

```text
callback ready
    ↓
sched_wakeup
    ↓
等待 CPU
    ↓
sched_switch
    ↓
callback_start
    ↓
callback_end
```

其中 `sched_switch` 可以进一步确定：

```text
CPU N:
    executor_thread_A
        ↓
    executor_thread_B
```

从而分析 callback 之间的 CPU 抢占和竞争。

## 5. 重启与 Kernel 更新

正常重启后不需要重新编译 LTTng。

如果希望开机自动加载：

```bash
echo lttng-tracer | sudo tee /etc/modules-load.d/lttng.conf
```

重启后检查：

```bash
lsmod | grep lttng
```

以及：

```bash
sudo lttng list --kernel
```

如果 kernel 版本发生变化，例如从：

```text
7.0.0-31-generic
```

变成其他版本，需要针对新的 kernel 重新编译：

```bash
cd ~/Downloads/lttng-modules-2.14.6

make clean
make

sudo make modules_install
sudo depmod -a
```

然后：

```bash
sudo modprobe lttng-tracer
```
