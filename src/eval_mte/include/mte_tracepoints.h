#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER eval

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "./mte_tracepoints.h"

#if !defined(_MTE_TP_H) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _MTE_TP_H

#include <lttng/tracepoint.h>

TRACEPOINT_EVENT(
    eval,
    algo_execute,
    TP_ARGS(const char*, node_id),
    TP_FIELDS(ctf_string(node_id, node_id))
)

TRACEPOINT_EVENT(
    eval,
    algo_working,
    TP_ARGS(const char*, node_id, int, core_id, long long, seg_us),
    TP_FIELDS(
        ctf_string(node_id, node_id)
        ctf_integer(int, core_id, core_id)
        ctf_integer(long long, seg_us, seg_us)
    )
)

TRACEPOINT_EVENT(
    eval,
    algo_complete,
    TP_ARGS(const char*, node_id),
    TP_FIELDS(ctf_string(node_id, node_id))
)

#endif /* _MTE_TP_H */

#include <lttng/tracepoint-event.h>