# ==============================================================================
# lttng2preempt.py - 从 LTTng trace 中提取 sched_switch 事件 (整理通用版)
# ==============================================================================
import os
import glob
import csv
import re
import argparse
import bt2

RESULTS_DIR = "./result/ROS"
FILTER_WORKER_ONLY = True
AFTER_US = 5000.0 * 1000

WORKER_PATTERNS = [
    r"^cie_container$",
    r"^mte_container$",
]


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
        if ctx is not None:
            self.vpid = int(_val(ctx.get("vpid")))
            self.vtid = int(_val(ctx.get("vtid")))
        else:
            self.vpid = 0
            self.vtid = 0
        try:
            pkt = ev.packet
            pc = pkt.context_field
            self.cpu = int(_val(pc.get("cpu_id"))) if pc is not None else -1
        except Exception:
            self.cpu = -1
        self.payload = _fields(ev.payload_field)


_WORKER_REGEX = [re.compile(p) for p in WORKER_PATTERNS]


def is_worker(comm):
    if not comm:
        return False
    for regex in _WORKER_REGEX:
        if regex.search(str(comm)):
            return True
    return False


def extract_switches(trace_dir):
    switches, t0 = [], None
    try:
        iterator = bt2.TraceCollectionMessageIterator(trace_dir)
    except Exception as e:
        print(f"  [Error] 无法打开 Trace 迭代器 ({trace_dir}): {e}")
        return [], None

    for msg in iterator:
        ev = getattr(msg, "event", None)
        if ev is None or ev.name != "sched_switch":
            continue
        
        e = EventView(ev, msg)
        if t0 is None or e.ts < t0:
            t0 = e.ts

        prev_tid = e.payload.get("prev_tid", "")
        next_tid = e.payload.get("next_tid", "")
        prev_comm = e.payload.get("prev_comm", "")
        next_comm = e.payload.get("next_comm", "")
        prev_state = e.payload.get("prev_state", "")

        prev_worker = is_worker(prev_comm)
        next_worker = is_worker(next_comm)

        if FILTER_WORKER_ONLY and not (prev_worker or next_worker):
            continue

        switches.append({
            "ts": e.ts,
            "cpu": e.cpu,
            "event_tid": e.vtid,
            "prev_tid": prev_tid,
            "prev_comm": prev_comm,
            "prev_state": prev_state,
            "next_tid": next_tid,
            "next_comm": next_comm,
            "prev_worker": prev_worker,
            "next_worker": next_worker,
        })
    return switches, t0


def export_switch_csv(switches, t0, output_path, after_us=0.0):
    if not switches:
        return 0

    rows = []
    for event in switches:
        ts = event["ts"]
        t_us = (ts - t0) / 1000.0
        if t_us < after_us:
            continue

        rows.append([
            ts,
            event["cpu"],
            event["prev_tid"],
            event["prev_comm"],
            event["prev_state"],
            event["next_tid"],
            event["next_comm"],
            int(event["prev_worker"]),
            int(event["next_worker"]),
            event["event_tid"],
        ])

    rows.sort(key=lambda row: row[0])

    with open(output_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "seq", "t_us", "cpu", "prev_tid", "prev_comm", "prev_state",
            "next_tid", "next_comm", "prev_is_worker", "next_is_worker", "event_tid"
        ])
        for seq, row in enumerate(rows):
            ts, cpu, prev_tid, prev_comm, prev_state, next_tid, next_comm, prev_worker, next_worker, event_tid = row
            writer.writerow([
                seq, round(ts / 1000.0, 3), cpu,
                prev_tid, prev_comm, prev_state,
                next_tid, next_comm,
                prev_worker, next_worker, event_tid
            ])

    return len(rows)


def process_one(folder, outdir, after_us):
    trace_path = os.path.join(folder, "trace")
    if not os.path.isdir(trace_path):
        return None, "没有 trace/ 子目录"

    name = os.path.basename(folder)
    print(f"  [读取] {name}")

    switches, t0 = extract_switches(trace_path)
    if t0 is None:
        return None, "没有找到 sched_switch 事件"

    output_path = os.path.join(outdir, f"{name}_preempt.csv")
    count = export_switch_csv(switches, t0, output_path, after_us=after_us)
    return os.path.basename(output_path), count


def main():
    parser = argparse.ArgumentParser(description="从 LTTng trace 中提取 sched_switch 抢占与切换信息")
    parser.add_argument("--results", default=RESULTS_DIR, help="实验结果目录")
    parser.add_argument("--outdir", default=RESULTS_DIR, help="CSV 输出目录")
    parser.add_argument("--after-us", type=float, default=AFTER_US, help="过滤前多少微秒的数据")
    parser.add_argument("--all", action="store_true", help="不进行 worker 过滤，输出所有 sched_switch")
    args = parser.parse_args()

    global FILTER_WORKER_ONLY
    if args.all:
        FILTER_WORKER_ONLY = False

    os.makedirs(args.outdir, exist_ok=True)

    print("========================================")
    print(" LTTng sched_switch exporter")
    print("========================================")
    print(f"results       : {args.results}")
    print(f"outdir        : {args.outdir}")
    print(f"worker filter : {FILTER_WORKER_ONLY}")
    print(f"after_us      : {args.after_us:g}\n")

    if not os.path.exists(args.results):
        print(f"❌ 错误: 根目录不存在: {args.results}")
        return

    folders = [
        os.path.join(args.results, d) for d in os.listdir(args.results)
        if os.path.isdir(os.path.join(args.results, d)) and not d.startswith("__")
    ]
    folders.sort()

    if not folders:
        print(f"在 {args.results}/ 下没有找到实验子目录。")
        return

    print(f"开始处理 {len(folders)} 个实验...")
    ok, total_rows = 0, 0

    for folder in folders:
        try:
            result, info = process_one(folder, args.outdir, args.after_us)
            if result is None:
                print(f"  [跳过] {os.path.basename(folder)}: {info}")
                continue
            ok += 1
            total_rows += info
            print(f"  [成功] {os.path.basename(folder)} -> {result} ({info} rows)")
        except Exception as exc:
            print(f"  [错误] {os.path.basename(folder)}: {exc}")

    print("========================================")
    print(f"完成：成功 {ok}/{len(folders)} 个实验，总计写入 {total_rows} 行")
    print("========================================")


if __name__ == "__main__":
    main()