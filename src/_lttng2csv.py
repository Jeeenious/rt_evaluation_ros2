#!/usr/bin/env python3
# coding: utf-8

# ==============================================================================
# lttng_export.py - 一次扫描，同时导出 timeline 与 preempt 两个 CSV，
#                   两者共享同一个 t0，时间轴严格对齐。
# ==============================================================================

import argparse
import csv
import os
import re
import bt2

RESULTS_DIR = "./result/CIE_FIFO_IPC"
AFTER_US = 2000.0 * 1000

# 断点续跑：默认跳过“本轮要求的 CSV 已生成且非空”的实验（便于中断后接着导，
# 也省掉最贵的 trace 解析）。需要全部重导时加 --rerun，或把此项改为 False。
SKIP_EXISTING = True

# ---- timeline 行为开关（对齐原 _lttng2timeline.py）----
USE_INTRAPROCESS = True
INCLUDE_FINISHED = True
INCLUDE_RELEASE = True
INCLUDE_WAKE_SLEEP = True

# ---- preempt 行为开关 ----
FILTER_WORKER_ONLY = True
WORKER_PATTERNS = [
    r"^cie_container$",
    r"^mte_container$",
]

_WORKER_REGEX = [re.compile(p) for p in WORKER_PATTERNS]


# ==============================================================================
# 通用字段读取
# ==============================================================================

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


def _int_or(value, default=0):
    try:
        return int(_val(value))
    except (TypeError, ValueError):
        return default


class EventView:
    def __init__(self, ev, msg):
        self.name = ev.name
        self.ts = int(msg.default_clock_snapshot.value)

        ctx = ev.common_context_field
        self.vpid = _int_or(ctx.get("vpid") if ctx is not None else None, 0)
        self.vtid = _int_or(ctx.get("vtid") if ctx is not None else None, 0)

        try:
            pc = ev.packet.context_field
            self.cpu = _int_or(pc.get("cpu_id") if pc is not None else None, -1)
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
        ev = getattr(msg, "event", None)
        if ev is None:
            continue
        yield EventView(ev, msg)


# ==============================================================================
# 事件分类
# ==============================================================================

def _classify_timeline(e):
    """返回 (kind, tag) 或 None。开关不满足时返回 None。"""
    n = e.name

    if n == "eval:algo_execute":
        return "execute", e.payload.get("node_id", "")
    if n == "eval:algo_complete":
        return "complete", e.payload.get("node_id", "")
    if n == "eval:algo_working":
        node_id = e.payload.get("node_id", "")
        core_id = e.payload.get("core_id", -1)
        seg_us = e.payload.get("seg_us", 0)
        return "working", f"{node_id} [CPU {core_id}, {seg_us} us]"

    if INCLUDE_FINISHED and n == "ros2:callback_end":
        return "finished", ""

    if INCLUDE_RELEASE and n == "ros2:callback_start":
            return "release", ""

    if INCLUDE_WAKE_SLEEP:
        if n == "ros2:rclcpp_executor_wait_for_work":
            return "sleep", ""
        if n == "ros2:rclcpp_executor_get_next_ready":
            return "wake", ""

    return None


def is_worker(comm):
    if not comm:
        return False
    return any(rx.search(str(comm)) for rx in _WORKER_REGEX)


# ==============================================================================
# 一次扫描收集两类事件
# ==============================================================================

def collect(trace_dir,
            want_timeline=True,
            want_preempt=True,
            worker_only=FILTER_WORKER_ONLY):
    timeline_events = []
    switch_events = []

    for e in _iter_events(trace_dir):
        if want_timeline:
            cls = _classify_timeline(e)
            if cls is not None:
                kind, tag = cls
                timeline_events.append({
                    "ts": e.ts, "tid": e.vtid, "cpu": e.cpu,
                    "kind": kind, "tag": tag,
                })

        if want_preempt and e.name == "sched_switch":
            prev_comm = e.payload.get("prev_comm", "")
            next_comm = e.payload.get("next_comm", "")
            prev_worker = is_worker(prev_comm)
            next_worker = is_worker(next_comm)

            if worker_only and not (prev_worker or next_worker):
                continue

            switch_events.append({
                "ts": e.ts, "cpu": e.cpu, "event_tid": e.vtid,
                "prev_tid": e.payload.get("prev_tid", ""),
                "prev_comm": prev_comm,
                "prev_state": e.payload.get("prev_state", ""),
                "next_tid": e.payload.get("next_tid", ""),
                "next_comm": next_comm,
                "prev_worker": prev_worker,
                "next_worker": next_worker,
            })

    return timeline_events, switch_events


# ==============================================================================
# 共享 t0：只在"最终保留的事件"上求最早 ts
# ==============================================================================

def shared_t0(timeline_events, switch_events):
    ts_values = [ev["ts"] for ev in timeline_events] + [ev["ts"] for ev in switch_events]
    return min(ts_values) if ts_values else None


# ==============================================================================
# 导出
# ==============================================================================

def write_timeline_csv(events, t0, output_path, after_us=0.0):
    if not events or t0 is None:
        return 0

    kept = [(ev, (ev["ts"] - t0) / 1000.0) for ev in events]
    kept = [(ev, t_us) for ev, t_us in kept if t_us >= after_us]
    kept.sort(key=lambda x: x[0]["ts"])

    with open(output_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "t_us", "cpu", "tid", "kind", "tag"])
        for seq, (ev, t_us) in enumerate(kept):
            w.writerow([seq, round(t_us, 3), ev["cpu"], ev["tid"], ev["kind"], ev["tag"]])
    return len(kept)


def write_preempt_csv(events, t0, output_path, after_us=0.0):
    if not events or t0 is None:
        return 0

    kept = [(ev, (ev["ts"] - t0) / 1000.0) for ev in events]
    kept = [(ev, t_us) for ev, t_us in kept if t_us >= after_us]
    kept.sort(key=lambda x: x[0]["ts"])

    with open(output_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "t_us", "cpu", "prev_tid", "prev_comm", "prev_state",
                    "next_tid", "next_comm", "prev_is_worker", "next_is_worker", "event_tid"])
        for seq, (ev, t_us) in enumerate(kept):
            w.writerow([
                seq, round(t_us, 3), ev["cpu"],
                ev["prev_tid"], ev["prev_comm"], ev["prev_state"],
                ev["next_tid"], ev["next_comm"],
                int(ev["prev_worker"]), int(ev["next_worker"]),
                ev["event_tid"],
            ])
    return len(kept)


# ==============================================================================
# 单实验
# ==============================================================================

def expected_outputs(name, outdir, want_timeline=True, want_preempt=True):
    """本轮要求生成的 CSV 路径列表。"""
    outs = []
    if want_timeline:
        outs.append(os.path.join(outdir, f"{name}_timeline.csv"))
    if want_preempt:
        outs.append(os.path.join(outdir, f"{name}_preempt.csv"))
    return outs


def outputs_ready(name, outdir, want_timeline=True, want_preempt=True):
    """本轮要求的 CSV 是否都已生成且非空（非空可排除写一半的残文件）。"""
    outs = expected_outputs(name, outdir, want_timeline, want_preempt)
    if not outs:
        return False
    return all(os.path.isfile(p) and os.path.getsize(p) > 0 for p in outs)


def process_one(folder, outdir, after_us,
                worker_only=FILTER_WORKER_ONLY,
                want_timeline=True,
                want_preempt=True):
    trace_path = os.path.join(folder, "trace")
    if not os.path.isdir(trace_path):
        return {"name": os.path.basename(folder), "error": "没有 trace/ 子目录"}

    name = os.path.basename(folder)
    print(f"  [读取] {name}")

    timeline_events, switch_events = collect(
        trace_path,
        want_timeline=want_timeline,
        want_preempt=want_preempt,
        worker_only=worker_only,
    )

    t0 = shared_t0(
        timeline_events if want_timeline else [],
        switch_events if want_preempt else [],
    )
    if t0 is None:
        return {"name": name, "error": "没有匹配到任何目标事件"}

    result = {"name": name, "t0": t0}

    if want_timeline:
        p = os.path.join(outdir, f"{name}_timeline.csv")
        result["timeline"] = (os.path.basename(p),
                              write_timeline_csv(timeline_events, t0, p, after_us))

    if want_preempt:
        p = os.path.join(outdir, f"{name}_preempt.csv")
        result["preempt"] = (os.path.basename(p),
                             write_preempt_csv(switch_events, t0, p, after_us))

    return result


# ==============================================================================
# 批量入口
# ==============================================================================

def run_export(results_dir=RESULTS_DIR,
               outdir=RESULTS_DIR,
               after_us=AFTER_US,
               worker_only=FILTER_WORKER_ONLY,
               want_timeline=True,
               want_preempt=True,
               skip_existing=SKIP_EXISTING):
    os.makedirs(outdir, exist_ok=True)

    if not os.path.exists(results_dir):
        print(f"❌ 错误: 根目录不存在: {results_dir}")
        return

    folders = [
        os.path.join(results_dir, d) for d in os.listdir(results_dir)
        if os.path.isdir(os.path.join(results_dir, d)) and not d.startswith("__")
    ]
    folders.sort()

    if not folders:
        print(f"在 {results_dir}/ 下没有找到实验子目录。")
        return

    tags = []
    if want_timeline: tags.append("timeline")
    if want_preempt:  tags.append("preempt")

    print("========================================")
    print(" LTTng exporter (shared t0)")
    print("========================================")
    print(f"results       : {results_dir}")
    print(f"outdir        : {outdir}")
    print(f"outputs       : {', '.join(tags)}")
    print(f"worker filter : {worker_only}")
    print(f"after_us      : {after_us:g}")
    print(f"intra-process : {USE_INTRAPROCESS}")
    print(f"skip existing : {skip_existing}")
    print(f"folders       : {len(folders)}\n")

    ok, skipped, total_tl, total_sw = 0, 0, 0, 0
    for folder in folders:
        name = os.path.basename(folder)

        if skip_existing and outputs_ready(name, outdir, want_timeline, want_preempt):
            skipped += 1
            print(f"  ⏭️  [跳过] {name}: CSV 已生成")
            continue

        try:
            r = process_one(folder, outdir, after_us,
                            worker_only=worker_only,
                            want_timeline=want_timeline,
                            want_preempt=want_preempt)
        except Exception as exc:
            print(f"  [错误] {name}: {exc}")
            continue

        if "error" in r:
            print(f"  [跳过] {name}: {r['error']}")
            continue

        parts = []
        if "timeline" in r:
            fn, n = r["timeline"]; parts.append(f"{fn} ({n} 行)"); total_tl += n
        if "preempt" in r:
            fn, n = r["preempt"];  parts.append(f"{fn} ({n} 行)"); total_sw += n
        ok += 1
        print(f"  [成功] {name}  t0={r['t0']}  ->  " + " | ".join(parts))

    print("========================================")
    print(f"完成：成功 {ok}，跳过 {skipped}（已生成），共 {len(folders)} 个实验")
    print(f"      timeline {total_tl} 行，preempt {total_sw} 行")
    if skipped:
        print("      提示：加 --rerun 可强制重导全部")
    print("========================================")


def main():
    ap = argparse.ArgumentParser(description="LTTng timeline + preempt 联合导出（共享 t0）")
    ap.add_argument("--results", default=RESULTS_DIR, help="实验结果目录")
    ap.add_argument("--outdir", default=RESULTS_DIR, help="CSV 输出目录")
    ap.add_argument("--after-us", type=float, default=AFTER_US, help="过滤掉前多少微秒")
    ap.add_argument("--all", action="store_true", help="preempt 不做 worker 过滤，输出全部 sched_switch")
    ap.add_argument("--no-timeline", action="store_true", help="只出 preempt")
    ap.add_argument("--no-preempt", action="store_true", help="只出 timeline")
    ap.add_argument("--rerun", action="store_true",
                    help="强制重导（默认跳过 CSV 已生成且非空的实验）")
    args = ap.parse_args()

    run_export(
        results_dir=args.results,
        outdir=args.outdir,
        after_us=args.after_us,
        worker_only=not args.all,
        want_timeline=not args.no_timeline,
        want_preempt=not args.no_preempt,
        skip_existing=not args.rerun,
    )


if __name__ == "__main__":
    main()