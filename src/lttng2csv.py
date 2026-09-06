#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""批量把 reports/ 下每个实验的 ros2_tracing trace 导出为“事件时间线” CSV。

列：tid, seq, kind, t_us, cpu, tag

【kind 与 ROS tracepoint 的对应关系】（我们命名 ↔ 真实事件 ↔ 语义）
  kind       对应 tracepoint                含义                           层级
  ---------  ---------------------------    ------------------------------  ------
  wake       rclcpp_executor_get_next_ready 线程从 wait 返回、取到就绪实体          [线程]
  release    rclcpp_executor_execute        该线程 executor 挑中此 job 的时刻      [节点/轮]
  execute    rclcpp callback_begin          do_work 开始执行                     [节点/轮]
  complete   rclcpp_publish                 本轮干完并对外通知/交给下游             [节点/轮]
  finished   rclcpp callback_end            回调返回收尾(发布后几 us)              [节点/轮]
  sleep      rclcpp_executor_wait_for_work  线程开始进入 wait（空闲开始）           [线程]

时序（按节点一轮）：release ≈ execute → complete(发布) → finished(返回)。
说明：
  - complete(发布) = 对外通知/交给下游，是跨节点“每轮”边界（算 makespan 用 complete）。
  - release 现指 executor 挑中该 job 的时刻，不代表上游到达，release≈execute（仅差
    executor 开销），因此不体现“在就绪队列等待”的时间。
注意：
  - sleep/wake 是“工作线程”的忙闲，不属于节点的一轮；一个忙期可连跑多个 job 才 sleep。
  - 一个节点一轮的完整执行 = [execute, finished]；对外边界(给下游/算 makespan)用 complete。

节点(tag)归属：do_work 只在自己线程上发布自己的输出话题 → 用“回调窗口内 + 同 vtid
的发布”认人（多线程窗口重叠、intra-process 下依然成立）。
前置条件：录制覆盖节点加载（ros2 trace start 先于 ros2 launch），否则 tag 无法解析。

用法：改下方配置区后直接运行 python3 lttng2csv.py；
可选 CLI 覆盖：--results / --outdir / --finished / --release / --wake-sleep / --after-us
"""
import argparse
import bisect
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
RESULTS_DIR = "reports"      # 每个实验一个文件夹（含 trace/ 子目录）
CSV_STATS_DIR = "results"    # 输出的时间线 CSV 放这里

# 要输出的 kind（都可开关）
INCLUDE_FINISHED = True      # finished = 该轮回调返回收尾（callback_end）；complete=发布=对外通知恒输出
INCLUDE_RELEASE = True       # release = rclcpp_callback_dispatch
INCLUDE_WAKE_SLEEP = True    # wake/sleep（线程级原始 tracing 事件）

# 裁掉 trace 开头的过渡/预热段（单位 us）。
# test.py 的 WARMUP_TIME=5s，即录制里前 5 秒是预热 → 这里设 5_000_000；
# 想保留全部就设 0。
AFTER_US = 5_000_000
# ==========================================

# SimComponent 的数据话题（p 输出形如 p<节点><下标>_<边>）
_DATA_TOPIC = re.compile(r'^/p\d+_\d+$')


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


def analyze(trace_dir, flags, min_us=0.0):
    node_handle2name = {}     # node_handle -> 'node_nn<i>'
    data_pub2node = {}        # data publisher_handle -> node name
    data_pub_topic = {}       # data publisher_handle -> topic
    node_sub_inputs = {}      # node_name -> set(输入数据话题)
    pub_by_topic = {}         # topic -> [发布时刻 ts]
    pub_events = []           # (ts, publisher_handle, vtid) 数据发布（intra 与 rcl 两种）
    target_pid = None
    open_job = {}             # (callback, vtid) -> job
    jobs = []                 # 待归属的 {start_ts, end_ts, vtid, cpu, callback}
    wait_per_vtid = {}        # vtid -> [(wait_for_work.ts, cpu), ...]
    ready_per_vtid = {}       # vtid -> [(get_next_ready.ts, cpu), ...]
    dispatch_per_vtid = {}    # vtid -> [(callback_dispatch.ts, cpu), ...]

    for e in _iter_events(trace_dir):
        n = e.name
        if n == 'ros2:rcl_node_init':
            nm = e.payload.get('node_name')
            if isinstance(nm, str) and nm.startswith('node_nn'):
                node_handle2name[e.payload['node_handle']] = nm
                if target_pid is None:
                    target_pid = e.vpid
            continue
        if target_pid is not None and e.vpid != target_pid:
            continue

        if n == 'ros2:rcl_publisher_init':
            nh = e.payload.get('node_handle')
            topic = e.payload.get('topic_name')
            nm = node_handle2name.get(nh)
            if nm and isinstance(topic, str) and _DATA_TOPIC.match(topic):
                data_pub2node[e.payload['publisher_handle']] = nm
                data_pub_topic[e.payload['publisher_handle']] = topic
        elif n == 'ros2:rcl_subscription_init':
            nh = e.payload.get('node_handle')
            topic = e.payload.get('topic_name')
            nm = node_handle2name.get(nh)
            if nm and isinstance(topic, str) and _DATA_TOPIC.match(topic):
                node_sub_inputs.setdefault(nm, set()).add(topic)
        elif n in ('ros2:rclcpp_intra_publish', 'ros2:rcl_publish'):
            h = e.payload.get('publisher_handle')
            if h in data_pub2node:
                pub_events.append((e.ts, h, e.vtid))
                pub_by_topic.setdefault(data_pub_topic[h], []).append(e.ts)
        elif n == 'ros2:rclcpp_executor_execute':
            dispatch_per_vtid.setdefault(e.vtid, []).append((e.ts, e.cpu))
        elif n == 'ros2:callback_start':
            callback = e.payload.get('callback')
            open_job[(callback, e.vtid)] = {
                'start_ts': e.ts,
                'vtid': e.vtid,
                'cpu': e.cpu,
                'callback': callback,
            }
        elif n == 'ros2:callback_end':
            key = (e.payload.get('callback'), e.vtid)
            if key in open_job:
                job = open_job.pop(key)
                job['end_ts'] = e.ts
                jobs.append(job)
        elif n == 'ros2:rclcpp_executor_wait_for_work':
            wait_per_vtid.setdefault(e.vtid, []).append((e.ts, e.cpu))
        elif n == 'ros2:rclcpp_executor_get_next_ready':
            ready_per_vtid.setdefault(e.vtid, []).append((e.ts, e.cpu))

    if target_pid is None:
        print('[失败] 未在 trace 中找到 node_nn* 节点初始化事件。'
              '请确认录制覆盖了 ros2 launch（先 ros2 trace start 再 launch）。')
        return None

    # 归属：do_work 只在自己线程上发布自己的输出话题 → 窗口内 + vtid 相同的发布认人
    pub_events.sort()
    pub_ts = [p[0] for p in pub_events]
    labeled = []
    for j in jobs:
        lo = bisect.bisect_right(pub_ts, j['start_ts'])
        hi = bisect.bisect_right(pub_ts, j['end_ts'])
        node = None
        for k in range(lo, hi):
            if pub_events[k][2] != j['vtid']:
                continue
            nm = data_pub2node.get(pub_events[k][1])
            if nm:
                node = nm
                break
        if node is None:
            continue
        j['node'] = node
        j['complete_ts'] = pub_events[k][0]     # 认人用的那次发布 = 该节点“对外通知完成”
        labeled.append(j)

    if not labeled:
        print('[警告] 未归属到任何节点 job（0 行）。请检查 trace 里是否包含发布事件。')
        return None

    # release = 实际 callback_dispatch 事件。
    # 仅按同一 vtid、且位于 callback_start 之前的最近 dispatch 关联。
    dispatch_ts = {}
    for vtid, pairs in dispatch_per_vtid.items():
        pairs.sort()
        dispatch_ts[vtid] = [t for t, _ in pairs]

    for j in labeled:
        ts_list = dispatch_ts.get(j['vtid'], [])
        k = bisect.bisect_right(ts_list, j['start_ts']) if ts_list else 0
        if k > 0:
            j['release_ts'] = ts_list[k - 1]

    return build_rows(labeled, wait_per_vtid, ready_per_vtid, flags, min_us)


def build_rows(jobs, wait_per_vtid, ready_per_vtid, flags, min_us):
    rows = []
    for j in jobs:
        tag = j['node'].replace('node_nn', 'n')
        rows.append((j['start_ts'], j['vtid'], j['cpu'], 'execute', tag))
        if 'complete_ts' in j:                       # complete = callback 内本节点 publish
            rows.append((j['complete_ts'], j['vtid'], j['cpu'], 'complete', tag))
        if flags.get('finished'):                  # finished = callback_end
            rows.append((j['end_ts'], j['vtid'], j['cpu'], 'finished', tag))
        if flags.get('release') and 'release_ts' in j:
            rows.append((j['release_ts'], j['vtid'], j['cpu'], 'release', tag))

    if flags['wake_sleep']:
        # sleep/wake 是线程级原始事件，与 callback jobs 独立。
        # 即使线程没有执行任何 callback，也保留完整的 sleep/wake 序列。
        thread_vtids = set(wait_per_vtid) | set(ready_per_vtid)

        for vtid in thread_vtids:
            waits = wait_per_vtid.get(vtid, [])
            readys = ready_per_vtid.get(vtid, [])

            events = (
                    [(ts, cpu, 'wait') for ts, cpu in waits] +
                    [(ts, cpu, 'ready') for ts, cpu in readys]
            )
            events.sort(key=lambda x: x[0])

            waiting = False
            for ts, cpu, kind in events:
                if kind == 'wait':
                    rows.append((ts, vtid, cpu, 'sleep', ''))
                    waiting = True
                elif kind == 'ready' and waiting:
                    rows.append((ts, vtid, cpu, 'wake', ''))
                    waiting = False

    if not rows:
        return None
    t0 = min(r[0] for r in rows)          # trace 起点（时间对齐基准）
    cand = []
    for ts, vtid, cpu, kind, tag in rows:
        if (ts - t0) / 1000.0 >= min_us:  # 裁掉预热/启动过渡段
            cand.append((ts, vtid, cpu, kind, tag))

    by_tid = {}
    for ts, vtid, cpu, kind, tag in cand:
        by_tid.setdefault(vtid, []).append((ts, cpu, kind, tag))

    # 输出顺序：按 tid 聚合（相同 tid 相邻），组内按真实时间升序；
    # seq = 该 tid 内按时间递增的序号。不再全局时间穿插 → 跨线程不再交错难读。
    flat = []
    for vtid in sorted(by_tid):
        for seq, (ts, cpu, kind, tag) in enumerate(sorted(by_tid[vtid])):
            flat.append([vtid, seq, kind, round((ts - t0) / 1000.0, 3), cpu, tag])
    return flat


def export_one(folder, outdir, flags, after_us):
    """处理一个实验文件夹，成功返回 (csv_name, rows)，失败返回 (None, 原因)。"""
    trace_path = os.path.join(folder, "trace")
    if not os.path.exists(trace_path):
        return None, "没有 trace/ 子目录"

    name = os.path.basename(folder)
    rows = analyze(trace_path, flags, min_us=after_us)
    if rows is None:
        return None, "解析失败（trace 需覆盖节点加载：先 ros2 trace start 再 launch）"

    out = os.path.join(outdir, f"{name}_timeline.csv")
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['tid', 'seq', 'kind', 't_us', 'cpu', 'tag'])
        w.writerows(rows)
    return os.path.basename(out), len(rows)


def main():
    ap = argparse.ArgumentParser(description='批量导出 reports/ 各实验的时间线 CSV')
    ap.add_argument('--results', default=RESULTS_DIR, help='实验结果目录（默认 reports）')
    ap.add_argument('--outdir', default=CSV_STATS_DIR, help='CSV 输出目录（默认 results）')
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

    folders = sorted(glob.glob(os.path.join(args.results, "feedback_*")))
    if not folders:
        print(f"在 {args.results}/ 下没有找到 feedback_* 实验文件夹。")
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

