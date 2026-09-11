# SLAM Project — Master Progress Doc
*Paste this whole file into a new chat for full context. No other file is required to continue work.*

## 1. What This Is
Custom lightweight 2D SLAM for **Milk-V Duo S** (ARM Cortex-A53, 512MB RAM, SD-card-only, no eMMC), replacing raw point-cloud SLAM with **line+arc feature** SLAM. A **STM32F411** coprocessor runs feature extraction on RPLIDAR A1 data and sends clean features over UART — the Duo S never sees raw LiDAR points. Novel architecture, built from scratch in C/Python (Python = 1:1 reference implementation to be ported to C; see §7).

**Why:** Hector/Cartographer/ROS2 too heavy for 512MB. Line features: 10-50/scan vs 360 raw points → lighter matching, smaller map (~5KB vs 256KB-1MB grid), more numerically stable (explicit orientation).

**Hardware:**
| Part | Role |
|---|---|
| RPLIDAR A1 | 360° LiDAR, UART 115200 |
| STM32F411 | Feature extraction (`fit_first.c`), must finish <100ms/scan (10Hz) |
| Milk-V Duo S | Runs SLAM (matching, pose, map). RAM budget: **150MB max** |
| PC | Dev/viz only (ROS2 Humble, RViz, Zenoh) |

**Hard constraints (never violate):**
- No Eigen/OpenCV/Ceres/ROS2 on embedded — plain C only, 2x2/3x3 linear algebra by hand
- `float` not `double` on STM32 (single-precision FPU), f-suffix every literal
- No dynamic allocation in embedded code
- Must cross-compile `aarch64-linux-gnu-g++`, test on QEMU before hardware
- Hough line distance can be **negative** — always sign-aware (`min(abs(d-e), abs(d+e))`)
- Everything is prototyped in Python first, 1:1-portable to C (pure functions, fixed-size loops, no classes-as-state where avoidable)

**Chat rules carried forward:** focus only on SLAM (not motors/nav/chassis); always produce working code, not descriptions; be honest about what won't work on constrained hardware; test-driven (unit tests alongside every module); simplest-working-first; when in doubt, write code.

---

## 2. Two-Phase Design
- **V1 (built, current focus):** local SLAM, no loop closure — matching → pose correction → map update, every scan.
- **V2 (not started):** pose-graph loop closure (`pose_graph.c`). Explicitly blocked until V1 is hardware-stable across stationary + translation + rotation.

## 3. Module Map (Python, in project root)
| File | Role | Status |
|---|---|---|
| `fit_first.c/.h` + `fit_first_ctypes.py` | STM32+PC feature extractor (line+arc), replaces split-and-merge | DONE, live on hardware @31fps |
| `messages.h` | Wire struct (`Feature` 48B, `FeaturePacket` 2408B), UART framing | DONE |
| `line_matcher.py` | Scan↔map feature matching (hard 1:1, Hough-space) | DONE (Python); C port pending |
| `map_manager.py` | Persistent map: `MapEntry` array, weighted-avg update, STATIC/DYNAMIC/UNCLASSIFIED decay | DONE (Python); C port pending |
| `pose_estimator.py` | Matched pairs → (dx,dy,dtheta) via Gauss-Newton, line+arc constraints | DONE |
| `soft_scan_matcher.py` | Soft-correspondence (Gaussian-weighted, no hard winner) GN step — replaces hard matching **inside** the fine refinement loop only | DONE |
| `correlative_match.py` | Coarse pose search (grid/pyramid scoring vs STATIC map) run **before** fine loop, breaks chicken-and-egg correspondence problem | DONE (2 variants exist — see §7.6) |
| `occupancy_grid.py` | PC-only: rasterizes line/arc map → OccupancyGrid for RViz. **No C port** (defeats the whole point of choosing line features over grids) | DONE, PC-only |
| `slam.py` | **Top-level orchestrator** — only file that knows all of the above. `SlamState.process_scan()` is the per-scan entry point | DONE, actively evolving |
| `lidar_visualizer.py` | ROS2 node: SDK→bins→features→SlamState→RViz topics | DONE, PC-only |
| `pose_graph.c` | V2 loop closure | NOT STARTED |

## 4. Per-Scan Pipeline (current, in `slam.py::process_scan`)
```
1. Initial guess = current_pose (no odometry hardware)
2. Coarse search (correlative_match.search): grid-score candidate poses
   vs STATIC map only → seeds working_pose. valid=False if map immature
   → falls back to unmodified current_pose (graceful degrade).
3. Fine refinement loop (up to MAX_ITERATIONS=8):
   a. transform scan → map frame using working_pose
   b. soft_scan_matcher.solve_soft_pose_step (soft-weighted GN, STATIC only)
   c. break on: low_weight / poor_conditioning / converged / exceeded_scan_budget
   d. clamp per-iteration step (MAX_ITER_STEP_TRANSLATION_M/ROTATION_RAD)
4. Confidence-gain damping (see §7.7) applied to fine-loop's total correction
5. Combine coarse+fine → pose_delta. Gate: magnitude-plausible? trend-consistent?
   coarse ambiguous? → decide delta_applied / delta_fully_trusted
6. current_pose += delta_applied ? delta : 0
7. Map update SKIPPED unless delta_fully_trusted (prevents bad pose writing
   map evidence that self-confirms next scan)
```

## 5. Key Runtime Concepts (must-know vocabulary for any new chat)
- **STATIC/DYNAMIC/UNCLASSIFIED**: map entry lifecycle. Only STATIC entries drive pose correction (moving objects/unconfirmed entries must never steer the pose).
- **Trend/probation** (`slam.py`): tracks recent accepted deltas; an off-trend delta still moves the pose (don't freeze on real jitter) but is blocked from writing map evidence for `TREND_PROBATION_SCANS` scans — prevents a wrong pose from self-validating via its own map writes.
- **Ambiguity gating**: `correlative_match` flags when a 2nd coarse candidate nearly ties the winner (e.g. rectangular-room symmetry). Ambiguous deltas get a longer `AMBIGUITY_PROBATION_SCANS` before being trusted to write the map — closes a real observed +30° heading lock-in.
- **Parallel-wall translation ambiguity**: a line only constrains translation *perpendicular* to itself. Fixed by folding ARC centre matches (isotropic, non-degenerate) into the same normal-equations solve.
- **Confidence-gain damping** (newest, §7.7): the fine loop's raw solve carries per-scan noise that was riding straight into `current_pose` unfiltered → visible TF jitter. Fixed via EMA-decayed, **weight-normalized** confidence gating.

## 6. Constants Quick-Reference (all in `slam.py` unless noted)
| Constant | Value | Purpose |
|---|---|---|
| `MAX_DELTA_TRANSLATION_M` | 0.20 (+1e-6 eps) | per-scan plausibility ceiling |
| `MAX_DELTA_ROTATION_RAD` | 20° | " |
| `MAX_ACCEPTABLE_RESIDUAL` | 0.15 | rejects wrong-correspondence deltas that look magnitude-OK |
| `TREND_PROBATION_SCANS` | 2 | scans blocked after an off-trend delta |
| `AMBIGUITY_PROBATION_SCANS` | 10 | longer block after ambiguous coarse seed |
| `MAX_ITER_STEP_TRANSLATION_M/ROTATION_RAD` | 0.08 / 8° | per-GN-iteration step clamp |
| `MAX_FINE_STATIC_ENTRIES` | 40 | fine-loop static-entry cap (perf) |
| `MAX_SCAN_FEATURES_FOR_MATCHING` | 18 | diversity-aware scan feature cap (perf) |
| `CONFIDENCE_EMA_ALPHA` | 0.35 | jitter-fix decay rate (~3-scan time const) |
| `MIN_USABLE_EIG_NORM` / `MIN_CONFIDENT_EIG_NORM` | 0.05 / 0.30 | normalized translation-confidence ramp |
| `MIN_USABLE_ROT_INFO_NORM` / `MIN_CONFIDENT_ROT_INFO_NORM` | 0.30 / 0.90 | normalized rotation-confidence ramp |
| `line_matcher.ANGLE_THRESH_RAD/DIST_THRESH_M` | 0.12 / 0.18 | tightened from 0.20/0.35 to stop wrong-wall matches |
| `map_manager.MIN_OBS_FOR_STATIC` | 30 | ~2s @ 7Hz before UNCLASSIFIED→STATIC |
| `map_manager.DECAY_SCANS` | 60 | ~8.5s miss before STATIC→DYNAMIC |

⚠️ Most thresholds above are **starting values**, explicitly flagged in-code as "tune once real logged hardware data exists." Not first-principles-derived.

## 7. Chronological Build History (bugs found → root cause → fix)

### 7.1 Base build (per `slam_build_prompt.md`)
messages.h → room_simulator → split_merge (later replaced by `fit_first`) → line_matcher → pose_estimator → map_manager → slam.c → cross-compile → STM32 port → hardware → tune → V2.

### 7.2 fit_first replaces split-and-merge
Recursive fit-first (line then arc, else split) beat split-and-merge; validated live: 16 lines + 9 arcs @31fps.

### 7.3 Pose estimator hardening (`slam_progress_update_pose_estimator.md`)
**Bugs:** (a) sliding along parallel walls → dx/dy jumped 1-2m (translation unobservable from parallel lines); (b) rotate+translate → correspondence flip-flopped between candidate walls; (c) probation self-corroborated (agreed with its own bad history).
**Fixes:** ARC centre constraints (isotropic, break parallel-wall singularity) folded into the same normal-equations solve; correspondence-churn check (<50% map-entry overlap between iterations → break); per-iteration step bound (previously only final total was bounded); probation now requires independent `correlative_match` corroboration, not just self-agreement.

### 7.4 Coarse search + trend gating (`slam_progress_update_coarse_search_and_trend_gating.md`)
**Bug 1:** map wrote garbage even when pose fully frozen — `skip_map_update` only checked `pose_delta.valid==True`, missed the "found nothing at all" (`valid=False`, mature map) case. Fixed: added `map_is_mature` check.
**Bug 2:** `correlative_match.search()` dropped `guess_x/guess_y` from scoring in both scalar and vectorized paths — silently correct only at guess=(0,0), confidently WRONG once robot moved from origin (i.e. almost always). Added regression tests (T8/T9) specifically at nonzero guess.
**Bug 3:** trend direction-check false-positived on ordinary noise — `DIRECTION_CHECK_MIN_MAG_M` (2cm) sat below the rig's real ~3-6.5cm stationary noise floor. Raised the threshold.
**Still open at the time:** rotation-only motion still broken (deferred to next session).

### 7.5 Soft correspondence (`soft_scan_matcher.py`)
Replaced the fine loop's **hard** 1:1 winner-take-all match (which could flip between two similar walls each GN iteration) with **Gaussian-weighted soft correspondence** over ALL nearby STATIC features at once — same philosophy as hector_slam's bilinearly-interpolated occupancy grid (no discrete "which cell" decision), applied to line/arc feature space instead of a grid (keeps RAM budget intact).

### 7.6 Performance pass (`slam_progress_old_algo_speedup.md`)
User had two `correlative_match.py` variants (pyramid vs. old two-layer coarse+greedy-walk). Sped up the old one instead of switching:
1. Layer-2 rotation cache (minor, <2% of runtime — not the real bottleneck)
2. `MAX_STATIC_ENTRIES_SEARCHED=40` cap (real fix, ported from pyramid)
3. Vectorized scoring (`_score_grid_vectorized`, numpy) — profiler showed old `_score_candidate` Python loop was 97%+ of runtime. Kept scalar version as cross-check reference (C port must mirror the **scalar** shape, numpy has no C equivalent).
4. `soft_scan_matcher` static-entry cap in `slam.py` (`MAX_FINE_STATIC_ENTRIES=40`) — fine stage scaled linearly with total map size (1.9ms@10 entries → 69ms@400, uncapped) → flat ~2-4ms after cap.
5. Diversity-aware scan-feature selection before matching (`MAX_SCAN_FEATURES_FOR_MATCHING=18`) — keeps all arcs, greedily picks non-duplicate-angle lines, fills leftover budget with duplicates only if diverse candidates run out. Map update (Step 5) still uses the **full unfiltered** scan — map completeness unaffected.
Result: 89.65ms→59.54ms avg, 160.59ms→72.69ms max @ n_static=60/27 features (real-log-representative). Widening search window untested at new headroom (flagged open item).
**Found but not fixed then:** float-boundary bug — a geometrically-exact 0.20m delta can land as `0.20000000000000037`, a hair over `MAX_DELTA_TRANSLATION_M`'s strict `<=`, wrongly rejected. Flagged as open item.

### 7.7 TF jitter fix — confidence-gain damping (this session, latest)
**Symptom:** visible TF vibration even on a near-stationary robot. Hardware log showed `OFF-TREND ... MAP WRITE BLOCKED` on nearly every scan with small, sign-flipping deltas.
**Root cause:** trend/probation gating (§7.4) only ever blocked **map writes**, never blocked applying the delta to `current_pose` itself — Step 6 applies any magnitude-plausible delta at full weight regardless of how weak its supporting evidence was (`final_weight`/`final_eig` swung 2.0→15.9 / 1.7→9.2 scan-to-scan in the log, each fully trusted).
**Rejected approach:** cosmetic smoothing (low-pass only the published/TF pose) — explicitly rejected by user; fix had to be in the core estimate.
**Design (approximates hector_slam's philosophy without its cost):** hector's own maintainers warn that naive per-scan Kalman fusion of the scan-match Hessian is wrong (consecutive scan errors are correlated, not independent) — they use covariance intersection instead. Full covariance intersection is out of scope (no-Eigen/no-malloc). Implemented instead: an EMA (`CONFIDENCE_EMA_ALPHA`) over the evidence itself (soft_scan_matcher's raw normal-equations entries `a11,a12,a22` and rotation resultant-vector length), decayed as a **matrix first**, then re-collapsed to an eigenvalue — not decaying an already-collapsed scalar.
**Critical calibration bug caught during build, not shipped naively:** raw eigenvalue/rotation-information scale with **feature count**, not just match quality — verified numerically (3-line clean match: eig≈1.0; 4-line: eig≈2.0; 25-feature dense hardware-scale: eig≈15.9 — all equally "good," 15x apart in magnitude). A single fixed absolute threshold cannot judge both regimes. **Fix: normalize by evidence weight before thresholding** — `norm_eig = decayed_eig / decayed_total_weight`, `norm_rot = decayed_rot / decayed_line_weight` (the latter is literally the circular-statistics "resultant length" R, bounded [0,1]). Verified: normalized eig ≈0.0 for genuinely degenerate (parallel-only) geometry, ≈0.33-0.5 for every clean geometry tested regardless of feature count 3-25; normalized rot ≈1.0 for all agreeing cases.
**Also fixed:** EMA cold-start bias (first real sample now snaps directly instead of decaying from a phantom zero seed, which would crush the very first post-bootstrap scan's gain to ~alpha regardless of quality); the float-boundary bug from §7.6 (surfaced directly once normalized gain converged a test delta almost exactly to `MAX_DELTA_TRANSLATION_M`) — fixed with a `1e-6` epsilon.
**Scope discipline:** gain applied ONLY to the fine loop's contribution (`total_dx/dy/dtheta`) before combining with the coarse seed — `correlative_match`'s own ambiguity/probation trust machinery (§7.4) is completely untouched, since hardware logs show coarse is invalid `(0,0,0)` on most scans (map not yet mature) and the fine loop is the dominant jitter source.
**Verified:** new self-test T12 measures actual variance reduction under alternating well-conditioned/near-parallel evidence with injected measurement noise — **~12x lower** applied-delta variance vs. the raw ungated solve (0.000181 → 0.000015). All 12 self-tests pass (`python3 slam.py`).
**Files touched:** `soft_scan_matcher.py` (returns 4 new diagnostic fields: `rotation_information`, `total_line_weight`, `a11`, `a12`, `a22`), `slam.py` (new EMA state + gain application + epsilon fix). `pose_estimator.py`, `line_matcher.py`, `map_manager.py`, `correlative_match.py` — **unchanged**.

## 8. Full Status Table
| Step | File | Status |
|---|---|---|
| 0 | fit_first | DONE, hardware-validated |
| 1 | messages.h | DONE |
| 2 | room_simulator | DONE |
| 3 | line_matcher | DONE (Python); C port pending |
| 4 | map_manager | DONE (Python); C port pending |
| 5 | lidar_visualizer | DONE, PC-only |
| 6 | pose_estimator | DONE, hardened (§7.3) |
| 6b | soft_scan_matcher | DONE (§7.5) |
| 6c | correlative_match | DONE, perf-tuned (§7.6) |
| 6d | confidence-gain jitter fix | DONE (§7.7), **not yet hardware-validated** |
| 7 | slam.c (full C port) | TODO |
| 8 | Cross-compile + QEMU | TODO |
| 9 | STM32 firmware port | TODO |
| 10 | Full hardware pipeline | TODO |
| 11 | pose_graph.c (V2 loop closure) | TODO, blocked on V1 stability |

## 9. Known Open Items (do not silently re-solve, ask user first)
1. **Rotation-only motion** — flagged broken in §7.4, status since then unconfirmed.
2. **Search-window widening** — rejected at old perf headroom (+50ms cost), untested at new ~40ms headroom post §7.6.
3. **§7.7 jitter fix is PC/Python-simulation-validated only** — needs real hardware log validation; `CONFIDENCE_EMA_ALPHA` and all `*_NORM` thresholds are starting values.
4. No C port exists yet for any module except `fit_first` — everything in §7 is Python reference implementation.
5. `occupancy_grid.py` is deliberately PC-only, never gets a C port (see file header) — don't "fix" this as an oversight.

## 10. How To Continue
- All modules are pure-function-style except `slam.py::SlamState` and `map_manager.py::MapManager` (own state, mutate in place, no malloc-equivalent growth beyond fixed arrays).
- Every module has a `python3 <file>.py` self-test at the bottom — run it after any edit, in a directory containing all sibling modules.
- Follow the Build Order in §1/§8 — do not skip to C port before Python reference is hardware-validated (constants are speculative until then).
- When editing, preserve the "why" comments in-code — this codebase relies heavily on documenting root-caused bugs inline so future edits don't reintroduce them.
