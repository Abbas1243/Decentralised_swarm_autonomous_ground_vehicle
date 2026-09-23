"""
correlative_match_ctypes.py
=============================
Drop-in ctypes wrapper around libcorrelative_match.so (built from
slam_core/correlative_match.c), matching correlative_match.py's own
public interface exactly:

    search(scan_features, map_entries, guess_x, guess_y, guess_theta,
           static_status=1) -> CoarseResult(dx, dy, dtheta, score,
                                             n_static, valid, ambiguous,
                                             second_score)

Same pattern already used for fit_first.c's PC wrapper
(fit_first_ctypes.py) — this is that pattern applied to
correlative_match.c so the REAL C implementation can be exercised by the
existing, unmodified Python stack (slam.py, bench_full_pipeline.py, the
self-test suites) instead of the pure-Python reference.

USAGE — swap this in for correlative_match.py in any existing script:

    import sys
    import correlative_match_ctypes
    sys.modules['correlative_match'] = correlative_match_ctypes
    import slam   # slam.py's `import correlative_match` now binds here

CoarseResult is intentionally the SAME namedtuple type correlative_match.py
defines (imported, not redefined) so `isinstance`/`._replace()` compatible
code (see correlative_match.py's own T11 self-test, which stubs
`correlative_match.search` and does `r._replace(...)`) keeps working
unchanged against this module's output too.

This wrapper does not attempt to reproduce correlative_match.py's
`last_search_total_s` / `last_search_layer1_s` / `last_search_layer2_s`
module-level timing attributes (those are perf_counter reads inside the
Python module) — slam.py already falls back to timing the call itself
when a `correlative_match` module doesn't expose `last_search_total_s`
(see SlamState.process_scan's `getattr(correlative_match,
"last_search_total_s", ...)` fallback), so no extra wiring is needed for
timing to work correctly through this wrapper. `last_search_n_static` IS
reproduced (cheap, and read directly in a few call sites), backed by the
C side's `corr_last_diagnostics`.
"""

import ctypes as ct
import os
import platform

from correlative_match import CoarseResult   # reuse the same namedtuple type

_HERE = os.path.dirname(os.path.abspath(__file__))
# Picks the aarch64 build automatically when run ON the Duo S itself
# (same "run the Python reference stack on-device for benchmarking"
# approach slam_progress_compact.md's bench_onboard.py already uses) --
# falls back to the host build otherwise. Both are built from the exact
# same correlative_match.c; only the compile target differs.
if platform.machine() in ("aarch64", "arm64"):
    _LIB_NAME = "libcorrelative_match_arm64.so"
else:
    _LIB_NAME = "libcorrelative_match.so"
_LIB_PATH = os.path.join(_HERE, _LIB_NAME)

_lib = ct.CDLL(_LIB_PATH)

FEAT_LINE = 0
FEAT_ARC = 1


# ---------------------------------------------------------------------------
# ctypes struct mirrors — field order/types must match correlative_match.h
# / messages.h exactly. These are NOT declared __attribute__((packed)) on
# the C side (only messages.h's wire-protocol Feature struct is packed,
# for UART byte-layout reasons) -- CorrMapEntry/CoarseResult use ctypes'
# default (natural) struct alignment, which matches the C compiler's
# default layout as long as both sides are built for the same platform,
# which they are here (both compiled for the host running this process).
# ---------------------------------------------------------------------------

class _CFeature(ct.Structure):
    """Mirrors messages.h's packed Feature struct (48 bytes)."""
    _pack_ = 1
    _fields_ = [
        ("type", ct.c_uint8),
        ("_pad", ct.c_uint8 * 3),
        ("angle", ct.c_float),
        ("distance", ct.c_float),
        ("t_start", ct.c_float),
        ("t_end", ct.c_float),
        ("cx", ct.c_float),
        ("cy", ct.c_float),
        ("r", ct.c_float),
        ("theta_start", ct.c_float),
        ("theta_end", ct.c_float),
        ("length", ct.c_float),
        ("quality", ct.c_uint8),
        ("_pad2", ct.c_uint8 * 3),
    ]


assert ct.sizeof(_CFeature) == 48, \
    f"_CFeature layout drifted from messages.h: sizeof={ct.sizeof(_CFeature)}"


class _CCorrMapEntry(ct.Structure):
    """Mirrors correlative_match.h's CorrMapEntry (not packed)."""
    _fields_ = [
        ("angle", ct.c_float),
        ("distance", ct.c_float),
        ("mx", ct.c_float),
        ("my", ct.c_float),
        ("status", ct.c_uint8),
        ("active", ct.c_bool),
    ]


class _CCoarseResult(ct.Structure):
    """Mirrors correlative_match.h's CoarseResult (not packed)."""
    _fields_ = [
        ("dx", ct.c_float),
        ("dy", ct.c_float),
        ("dtheta", ct.c_float),
        ("score", ct.c_float),
        ("n_static", ct.c_int),
        ("valid", ct.c_bool),
        ("ambiguous", ct.c_bool),
        ("second_score", ct.c_float),
    ]


_lib.correlative_search.argtypes = [
    ct.POINTER(_CFeature), ct.c_int,
    ct.POINTER(_CCorrMapEntry), ct.c_int,
    ct.c_float, ct.c_float, ct.c_float,
    ct.c_uint8,
]
_lib.correlative_search.restype = _CCoarseResult


# ---------------------------------------------------------------------------
# Python dict / MapEntry  ->  ctypes struct conversion
# ---------------------------------------------------------------------------

def _to_c_feature(feat):
    f = _CFeature()
    if feat["type"] == "line":
        f.type = FEAT_LINE
        f.angle = feat["angle"]
        f.cx = feat["x1"]
        f.cy = feat["y1"]
        f.t_start = feat["x2"]
        f.t_end = feat["y2"]
    elif feat["type"] == "arc":
        f.type = FEAT_ARC
        f.cx = feat["cx"]
        f.cy = feat["cy"]
        f.r = feat["r"]
    else:
        f.type = 0xFF   # unknown -- C side skips anything not FEAT_LINE/FEAT_ARC
    f.quality = int(feat.get("quality", 100))
    return f


def _to_c_map_entry(entry):
    e = _CCorrMapEntry()
    e.angle = entry.angle
    e.distance = entry.distance
    e.mx = entry.mx
    e.my = entry.my
    e.status = entry.status
    e.active = bool(entry.active)
    return e


# ---------------------------------------------------------------------------
# Public API -- same signature as correlative_match.py's search()
# ---------------------------------------------------------------------------

last_search_n_static = 0


def search(scan_features, map_entries, guess_x, guess_y, guess_theta, static_status=1):
    global last_search_n_static

    n_scan = len(scan_features)
    c_scan = (_CFeature * max(n_scan, 1))()
    for i, feat in enumerate(scan_features):
        c_scan[i] = _to_c_feature(feat)

    n_map = len(map_entries)
    c_map = (_CCorrMapEntry * max(n_map, 1))()
    for i, entry in enumerate(map_entries):
        c_map[i] = _to_c_map_entry(entry)

    result = _lib.correlative_search(
        c_scan, n_scan, c_map, n_map,
        ct.c_float(guess_x), ct.c_float(guess_y), ct.c_float(guess_theta),
        ct.c_uint8(static_status),
    )

    last_search_n_static = result.n_static

    return CoarseResult(
        dx=result.dx, dy=result.dy, dtheta=result.dtheta,
        score=result.score, n_static=result.n_static,
        valid=bool(result.valid), ambiguous=bool(result.ambiguous),
        second_score=result.second_score,
    )
