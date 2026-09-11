"""批量把 reports/ 下每个实验的 ros2_tracing trace 导出为“事件时间线” CSV。

列：tid, seq, kind, t_us, cpu, tag（输出按 cpu/core 聚合，组内按时间排序）

【kind 与 ROS tracepoint 的对应关系】（我们命名 ↔ 真实事件 ↔ 语义）
  kind       对应 tracepoint                含义                           层级
  ---------  ---------------------------    ------------------------------  ------
  wake       rclcpp_executor_get_next_ready 线程从 wait 返回、取到就绪实体          [线程]
  release
  -> !IPC    rclcpp_executor_execute        该线程 executor 挑中此 job 的时刻      [节点/轮]
  -> IPC     rclcpp:callback_start          ~ 该线程 executor 挑中此 job 的时刻    [节点/轮]
  execute    eval:algo_execute[自定义]       do_work 执行（已经收到数据）           [节点/轮]
  complete   eval:algo_complete[自定义]      do_work 算完（尚未发布完毕）           [节点/轮]
  finished   rclcpp:callback_end            回调返回收尾                          [节点/轮]
  sleep      rclcpp_executor_wait_for_work  线程开始进入 wait（空闲开始）           [线程]

  注意： IPC 下 release -> execute 延迟低但并不能说明算法包装成本低，仅仅是原生框架里不方便取到 execute 的时刻，所以才用 callback_start 代替。
"""

import argparse
import csv
import glob
import os
import re
import sys
from collections import Counter

import bt2

# ================= 配置区 =================
# 所有参数都集中在这里改，然后直接运行：python3 lttng2csv.py
# ------------------------------------------------------------
# 实验与输出目录
RESULTS_DIR = "./result/CIE_FIFO_IPC_nuc12"      # 每个实验一个文件夹（含 trace/ 子目录）

# USE_INTRAPROCESS: 是否启用 ROS 2 内部进程通信
USE_INTRAPROCESS = True

# 要输出的 kind（都可开关）
INCLUDE_FINISHED = True      # finished = 该轮回调返回收尾（callback_end）；complete=发布=对外通知恒输出
INCLUDE_RELEASE = True       # release = rclcpp_callback_dispatch
INCLUDE_WAKE_SLEEP = True    # wake/sleep（线程级原始 tracing 事件）

# 裁掉 trace 开头的预热段（单位 us）
AFTER_US = 2000 * 1000  # 2 seconds (test.py trace 启动需要 1.5s)
# ==========================================


def _val(field):
    """统一取值：有符号/无符号整型与字符串字段的 API 略有差异。"""
    try:
        return field.value
    except Exception:
        pass
    s = str(field)
    try:
        return int(s)
    except (TypeError, ValueError):
        return s


def _fields(struct):
    return {k: _val(struct.get(k)) for k in struct.keys()} if struct is not None else {}


class EventView:
    def __init__(self, ev, msg):
        self.ev = ev
        self.name = ev.name
        self.ts = int(msg.default_clock_snapshot.value)
        ctx = ev.common_context_field
        self.vpid = int(_val(ctx.get('vpid'))) if ctx is not None else 0
        self.vtid = int(_val(ctx.get('vtid'))) if ctx is not None else 0
        pkt = ev.packet
        try:
            pc = pkt.context_field
            self.cpu = int(_val(pc.get('cpu_id'))) if pc is not None else -1
        except Exception:
            self.cpu = -1
        self.payload = _fields(ev.payload_field)


def _iter_events(trace_dir):
    for msg in bt2.TraceCollectionMessageIterator(trace_dir):
        ev = getattr(msg, 'event', None)
        if ev is None:
            continue
        yield EventView(ev, msg)


def analyze(trace_dir, min_us=0.0):
    rows = []

    for e in _iter_events(trace_dir):
        n = e.name

        if n == 'eval:algo_execute':
            kind = 'execute'
            tag = e.payload.get('node_id', '')

        elif n == 'eval:algo_complete':
            kind = 'complete'
            tag = e.payload.get('node_id', '')

        elif n == 'sched_switch':
            kind = 'switch'

            prev_tid = e.payload.get('prev_tid', '')
            next_tid = e.payload.get('next_tid', '')
            prev_comm = e.payload.get('prev_comm', '')
            next_comm = e.payload.get('next_comm', '')
            prev_state = e.payload.get('prev_state', '')

            tag = (
                f'{prev_comm}[{prev_tid}] -> '
                f'{next_comm}[{next_tid}]'
            )


        elif n == 'ros2:callback_end':
            kind = 'finished'
            tag = ''

        elif not USE_INTRAPROCESS and n == 'ros2:rclcpp_executor_execute':
            kind = 'release'
            tag = ''

        elif USE_INTRAPROCESS and n == 'ros2:callback_start':
            kind = 'release'
            tag = ''

        elif n == 'ros2:rclcpp_executor_wait_for_work':
            kind = 'sleep'
            tag = ''

        elif n == 'ros2:rclcpp_executor_get_next_ready':
            kind = 'wake'
            tag = ''

        else:
            continue

        rows.append((
            e.ts,
            e.vtid,
            e.cpu,
            kind,
            tag,
        ))

    if not rows:
        return None

    t0 = min(r[0] for r in rows)

    cand = []
    for ts, vtid, cpu, kind, tag in rows:
        if (ts - t0) / 1000.0 >= min_us:
            cand.append((ts, vtid, cpu, kind, tag))

    flat = []
    for seq, (ts, vtid, cpu, kind, tag) in enumerate(cand):

        flat.append([
            vtid,
            seq,
            kind,
            round((ts - t0) / 1000.0, 3),
            cpu,
            tag,
        ])

    return flat


def export_one(folder, outdir, flags, after_us):
    """处理一个实验文件夹，成功返回 (csv_name, rows)，失败返回 (None, 原因)。"""
    trace_path = os.path.join(folder, "trace")
    if not os.path.exists(trace_path):
        return None, "没有 trace/ 子目录"

    name = os.path.basename(folder)
    rows = analyze(trace_path, min_us=after_us)
    if rows is None:
        return None, "解析失败（请确认 trace 中是否包含 eval:algo_execute/algo_complete）"

    out = os.path.join(outdir, f"{name}_timeline.csv")
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['tid', 'seq', 'kind', 't_us', 'cpu', 'tag'])
        w.writerows(rows)
    return os.path.basename(out), len(rows)


def main():
    ap = argparse.ArgumentParser(description='批量导出 reports/ 各实验的时间线 CSV')
    ap.add_argument('--results', default=RESULTS_DIR, help='实验结果目录')
    ap.add_argument('--outdir', default=RESULTS_DIR, help='CSV 输出目录')
    ap.add_argument('--finished', action='store_true', default=INCLUDE_FINISHED)
    ap.add_argument('--release', action='store_true', default=INCLUDE_RELEASE)
    ap.add_argument('--wake-sleep', action='store_true', default=INCLUDE_WAKE_SLEEP)
    ap.add_argument('--after-us', type=float, default=AFTER_US)
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    flags = {'finished': args.finished, 'release': args.release,
             'wake_sleep': args.wake_sleep}

    print("配置："
          f" finished={flags['finished']}, release={flags['release']},"
          f" wake_sleep={flags['wake_sleep']}, after_us={args.after_us:g}")

    folders = sorted(glob.glob(os.path.join(args.results, "*")))
    if not folders:
        # 如果没有 * 文件夹，尝试匹配 results 下的所有子目录
        folders = [os.path.join(args.results, d) for d in os.listdir(args.results)
                   if os.path.isdir(os.path.join(args.results, d))]
        folders = sorted(folders)

    if not folders:
        print(f"在 {args.results}/ 下没有找到包含 trace 的实验文件夹。")
        return

    print(f"开始批量导出 {len(folders)} 个实验 -> {args.outdir}/")
    ok = 0
    for folder in folders:
        out, info = export_one(folder, args.outdir, flags, args.after_us)
        if out:
            ok += 1
            print(f"  [成功] {os.path.basename(folder)} -> {out} ({info} 行)")
        else:
            print(f"  [跳过] {os.path.basename(folder)}: {info}")
    print(f"\n完成。成功 {ok} / {len(folders)}，CSV 在 {args.outdir}/")


if __name__ == "__main__":
    main()
