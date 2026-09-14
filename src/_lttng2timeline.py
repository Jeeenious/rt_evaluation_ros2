# ==============================================================================
# _lttng2timeline.py - 导出 ROS2 任务执行时间线 CSV (整理版)
# ==============================================================================
import argparse
import csv
import glob
import os
import bt2

RESULTS_DIR = "./result/ROS"
USE_INTRAPROCESS = True
INCLUDE_FINISHED = True
INCLUDE_RELEASE = True
INCLUDE_WAKE_SLEEP = True
AFTER_US = 5000 * 1000


def _val(field):
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
        try:
            pkt = ev.packet
            pc = pkt.context_field
            self.cpu = int(_val(pc.get('cpu_id'))) if pc is not None else -1
        except Exception:
            self.cpu = -1
        self.payload = _fields(ev.payload_field)


def _iter_events(trace_dir):
    try:
        iterator = bt2.TraceCollectionMessageIterator(trace_dir)
    except Exception as e:
        print(f"  [Error] 无法打开 Trace 迭代器 ({trace_dir}): {e}")
        return
    for msg in iterator:
        ev = getattr(msg, 'event', None)
        if ev is None:
            continue
        yield EventView(ev, msg)


def analyze(trace_dir, min_us=0.0):
    rows = []
    for e in _iter_events(trace_dir):
        n = e.name

        if n == 'eval:algo_execute':
            kind, tag = 'execute', e.payload.get('node_id', '')
        elif n == 'eval:algo_complete':
            kind, tag = 'complete', e.payload.get('node_id', '')
        elif n == 'eval:algo_working':
            node_id = e.payload.get('node_id', '')
            core_id = e.payload.get('core_id', -1)
            seg_us = e.payload.get('seg_us', 0)
            kind, tag = 'working', f'{node_id} [CPU {core_id}, {seg_us} us]'
        elif n == 'ros2:callback_end':
            kind, tag = 'finished', ''
        elif not USE_INTRAPROCESS and n == 'ros2:rclcpp_executor_execute':
            kind, tag = 'release', ''
        elif USE_INTRAPROCESS and n == 'ros2:callback_start':
            kind, tag = 'release', ''
        elif n == 'ros2:rclcpp_executor_wait_for_work':
            kind, tag = 'sleep', ''
        elif n == 'ros2:rclcpp_executor_get_next_ready':
            kind, tag = 'wake', ''
        else:
            continue

        rows.append((e.ts, e.vtid, e.cpu, kind, tag))

    if not rows:
        return None

    t0 = min(r[0] for r in rows)
    cand = []
    for ts, vtid, cpu, kind, tag in rows:
        rel_us = (ts - t0) / 1000.0
        if rel_us >= min_us:
            cand.append((ts, vtid, cpu, kind, tag))

    flat = []
    for seq, (ts, vtid, cpu, kind, tag) in enumerate(cand):
        flat.append([
            seq,
            round(ts / 1000.0, 3),
            cpu,
            vtid,
            kind,
            tag,
        ])
    return flat


def export_one(folder, outdir, after_us):
    trace_path = os.path.join(folder, "trace")
    if not os.path.exists(trace_path):
        return None, "没有 trace/ 子目录"

    name = os.path.basename(folder)
    rows = analyze(trace_path, min_us=after_us)
    if rows is None:
        return None, "解析失败（未匹配到有效的 tracepoint 事件）"

    out = os.path.join(outdir, f"{name}_timeline.csv")
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['seq', 't_us', 'cpu', 'tid', 'kind', 'tag'])
        w.writerows(rows)
    return os.path.basename(out), len(rows)


def main():
    print("========================================")
    print(" ROS2 Timeline Exporter")
    print("========================================")

    ap = argparse.ArgumentParser(description='批量导出 ROS2 实验时间线 CSV')
    ap.add_argument('--results', default=RESULTS_DIR, help='实验结果目录')
    ap.add_argument('--outdir', default=RESULTS_DIR, help='CSV 输出目录')
    ap.add_argument('--after-us', type=float, default=AFTER_US)
    args = ap.parse_args()

    print(f"results       : {args.results}")
    print(f"outdir        : {args.outdir}")
    print(f"after_us      : {args.after_us:g}")

    os.makedirs(args.outdir, exist_ok=True)

    if not os.path.exists(args.results):
        print(f"❌ 错误: 根目录不存在: {args.results}")
        return

    folders = [
        os.path.join(args.results, d) for d in os.listdir(args.results)
        if os.path.isdir(os.path.join(args.results, d)) and not d.startswith("__")
    ]
    folders.sort()

    if not folders:
        print(f"在 {args.results}/ 下没有找到实验文件夹。")
        return

    print(f"开始批量导出 {len(folders)} 个实验 -> {args.outdir}/")
    ok = 0
    for folder in folders:
        out, info = export_one(folder, args.outdir, args.after_us)
        if out:
            ok += 1
            print(f"  [Success] {os.path.basename(folder)} -> {out} ({info} 行)")
        else:
            print(f"  [跳过] {os.path.basename(folder)}: {info}")
            
    print(f"\n完成。成功 {ok} / {len(folders)}")


if __name__ == "__main__":
    main()