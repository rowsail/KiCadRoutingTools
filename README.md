<p align="center">
  <img src="kicad_routing_plugin/icon_512_text.png" alt="KiCadRoutingTools" width="200">
</p>

# KiCad Routing Tools — rowsail fork

This is a fork of **[drandyhaas/KiCadRoutingTools](https://github.com/drandyhaas/KiCadRoutingTools)**, the Rust-accelerated A* autorouter for KiCad 9 and 10, by Dr. Andy Haas. All credit for the tool itself goes to him. If you find it useful, please consider [supporting the original project](https://ko-fi.com/drandyhaas).

The fork is based on upstream **v0.22.1** (`023d3f79`). It adds 33 commits: speed-ups (a full route about 3x faster with identical output), bug fixes (shorts, clearance errors, mismatched diff-pair widths), and new post-route options (via-in-pad control, small-via replacement, power-neck widening, dangling-copper clean-up). They are listed and explained below. Upstream has since moved on (v0.23.0 and later); these changes have not yet been rebased onto it.

- **Installing and using the tool:** see the original **[README-upstream.md](README-upstream.md)**. Installation is the same, but use this repository's URL. The Rust router must be rebuilt from this source (`python3 build_router.py`), because several changes are in Rust.
- **The changes:** the rest of this page. It was written as a report for the upstream maintainer, so "you" means him and "I" means the author of these changes. Patch numbers 0001–0033 match, in order, the commits on `main` after `v0.22.1` (`git log --reverse v0.22.1..main`).
- **Licence:** MIT, as upstream (see [LICENSE](LICENSE)).

---

The changes fall into four kinds:

- **Exact speed-ups.** The routed board is identical to v0.22.1's apart from UUIDs. Together they make a full CLI route of my test board about 3x faster (417 s to 141 s for 0001–0007, more with 0011–0013 and 0017).
- **Bug fixes.** These change output, because the old output was wrong (shorts, clearance errors, mismatched widths).
- **New options.** Each is an opt-in, or on by default where noted. Leaving it off gives the same output as before.
- **Local preferences.** Defaults I changed for my own use (marked ★). You may well not want these upstream.

## Index

| Patch | What | Kind | Default | Changes routed output? |
|---|---|---|---|---|
| 0001 | underpad: `_carve_foreign` byte masks | speed | — | no |
| 0002 | parser: loose-end buckets | speed | — | no |
| 0003 | net_rescue: restore build records on a cache hit | fix | — | only on rescue-map cache hits |
| 0004 | grid_router: release the GIL during A* | speed / UI | — | no |
| 0005 | grid_router: dense refcount storage | speed | — | no |
| 0006 | grid_router: Rust rasterizers | speed | on (`KICAD_RUST_RASTER=0` off) | no |
| 0007 | parallel rescue ladder (CLI) | speed | on (`KICAD_RESCUE_PARALLEL=0` off) | no |
| 0008 | plane join counter shows the join number | cosmetic | — | no |
| 0009 | dialog status line: live tallies | cosmetic | — | no |
| 0010 | speculative bare-pad fanout rescue | speed | on | no |
| 0011 | span stamping, tile cache, bitmap growth | speed | — | no |
| 0012 | exact Rust foreign-clearance checks | speed | — | no |
| 0013 | incremental plane-fragility publish | speed | — | no |
| 0014 | GPU global plan (PathFinder on CUDA) | option | off | only when enabled |
| 0015 | small-via rung saw no base copper on rescue maps | **fix (shorts)** | — | yes: removes shorts |
| 0016 | GPU wavefront search (Lee + guided A*) | option | on when a GPU is usable | yes (different, valid routes) |
| 0017 | fast wide-margin segment checks | speed | — | no |
| 0018 | clean-up: redundant leaf copper | option / fix | on (`KICAD_PRUNE_REDUNDANT=0` off) | yes: removes dead copper |
| 0019 | clean-up: junk copper, layer-aware pins | fix | on | yes: removes dead copper |
| 0020 | Board Setup via minimums declare the fab's via capability | fix | — | yes, when net-class vias are below the tier floor |
| 0021 | GPU rip-up planning | option | off | only when enabled |
| 0022 | net-class clearances for GND return vias and headless plan runs | **fix (DRC)** | — | yes: removes clearance errors |
| 0023 | ★ "Fix DRC settings after routing" opt-in | local pref | off | no (project rules untouched) |
| 0024 | diff pair pitch at drawn width; diff search weight 1.3; ★ turn cost 3000 | fix + ★ | — | yes |
| 0025 | clean-up: shadow tracks | fix | on | yes: removes overlapping copper |
| 0026 | grow vias in small pads | option | off | only when enabled |
| 0027 | weld stranded pour islands with a via | fix / option | on | yes, only when KiCad reports unconnected items |
| 0028 | replace small-drill vias by re-routing; welder stub escapes | option | off | only when enabled |
| 0029 | ★ small-via replacement on by default | local pref | on | yes, on boards with small-drill vias |
| 0030 | Board Setup minimum via is a hard floor; repair boxed-in plane pads | **fix** + option | — / on with 0028 | yes: no via below Board Setup's minimum |
| 0031 | small-via pass sweeps dangling copper reported by KiCad DRC | fix | on with 0028/0029 | yes: removes dangling vias/stubs |
| 0032 | widen power-net necks after routing (DRC-checked, per net) | option | on with 0028/0029 | yes: wider power tracks |
| 0033 | honour via-in-pad off everywhere (site policy in headless runs, return vias, welder, small-via pass); exposed centre pads exempt | fix | follows project | yes: no vias in normal pads |

Sections 0001–0011 below come from the first round and include the measurements. Later patches have their own sections further down.

## 0001–0011: results

Measured on a private 2-IC board with an ESP32-S3 (QFN-56) and a motor driver, routed from unrouted with `py_router/route.py <board> --output <out>` (CLI defaults). Times are wall clock, unprofiled, on a 32-core Linux box with Python 3.13.

| After patch | Full route | Change |
|---|---|---|
| v0.22.1 | 417 s | — |
| 0001 `_carve_foreign` byte masks | 213 s | ×1.96 |
| 0002 parser loose-end buckets | – | measured together with 0006 |
| 0003 rescue-map record fix | – | no speed change on its own |
| 0004 release the GIL | 212 s | (UI responsiveness, not speed) |
| 0007 parallel rescue ladder (CLI) | 186 s | |
| 0005 dense refcounts | 172 s | |
| 0006 Rust rasterizers (+0002) | 141 s | |

(Measured in the order I wrote them, which isn't the series order. Peak RSS is about 2.2 GB throughout.)

## How each patch was checked

After every step, the routed board was compared with v0.22.1's output, with `(uuid "…")` normalised. Every one was identical. For the parallel rescue (0007) I also compared the full log and `JSON_SUMMARY` with a serial run. They differ only in timing fields and the env-knob listing, plus the "Blocking obstacles" name order, which already varies between two serial runs.

Unit-level differential tests against the code being replaced, all identical:

| Patch | Test |
|---|---|
| 0001 | 775k random membership probes, mask view vs `home - c_all - c_lay` |
| 0002 | 4,000 random endpoint sets: duplicates, points at ±tol (×0.999/1.0/1.001), coordinates around 0 where truncating division makes bucket 0 double-width |
| 0005 | Rust unit test: `RefMap` vs `FxHashMap` over 400k random inc/dec/get operations, including the overflow path |
| 0006 | capsules 60k (on-grid, zero-length, axis/45°, tie-prone margins such as 0.3/2+0.25 on a 0.1 grid); pads 40k (rounded, rotated incl. ±90°/180°/~1e-10°, sub-cell offsets; same cells **in the same order**); disks 180k |

Your tests: these 17 test files pass on both v0.22.1 and the branch (49 tests):
`test_134_partial_restore_refreshes_obstacles`, `test_369_writer_parser_fixes`, `test_456_courtyard_parser`, `test_505_edge_contour_reclassify`, `test_556_obstacle_build`, `test_612_raster_and_primary_fallbacks`, `test_619_underpad_stub_vs_erased_copper`, `test_628_milled_contour_legality`, `test_653_env_knob_logging`, `test_798_registrar_flags`, `test_864_underpad_carve`, `test_895_boundary_criteria`, `test_bga_fanout_underpad_dense_rail`, `test_bga_fanout_underpad`, `test_bga_underpad_diff`, `test_doc_flag_liveness`, `test_net_rescue`.

I have **not** run your full suite or your corpus. It would be worth running both, since my evidence comes from one board.

## The patches

**0001 underpad: keep `_carve_foreign`'s foreign stamps as byte masks.**
The biggest win. `_carve_foreign` built each nearby foreign disk as a frozenset of `(ix, iy)` tuples and intersected it with `home`: about 280M tuples per route, and 268 s of a 584 s profile. All consumers only test membership, always after `c in home`. So the carve is now a `_CellMask` (one byte per cell, `_Occ.grid` layout, filled by column spans), and `_eff_exempt` returns a membership-only `_HomeExempt` view. `_carve_foreign`: 268 s → 10 s.

**0002 kicad_parser: count contour loose ends with the endpoint buckets.**
`_chain_segments_into_contours` counted each endpoint's partners against every endpoint, which came to 21.5M `approx_equal` calls per route on an outline with arcs. It now uses the same `2*tol` buckets the union-find step already uses, giving the same count and the same loose ends in the same order.

**0003 net_rescue: restore a cached rescue map's build records on a hit.** ⚠ *Behaviour change on cache hits only.*
`build_base_obstacle_map` writes `_net_tie_lift` / `_net_tie_price` (and the own-pad rows) onto the board, and `routing_context` reads the net-tie ones back from the board. `_pristine_rescue_map` caches the own-pad rows with the map for exactly this reason (#977), but on a hit the net-tie records were still whatever the last build had written, possibly another rung's. This patch stores all four records in the cache entry and restores them on a hit. My board never hits that cache (249/249 misses), so I couldn't observe a difference. On boards that do hit it, routes may legitimately change. Please check this one against your intent.

**0004 grid_router: release the GIL during the A* searches.**
`py.allow_threads` around `route_multi`, `route_with_frontier`, `route_pose` and `route_pose_with_frontier`. It compiles unchanged, because the types are already `Send`, and pyo3's borrow flag still guards the pyclass refs. The point is mainly the KiCad dialog, which routes on a thread and froze the UI's Python for every search: main-thread loop iterations during a 1.4 s search went from about 24k to 6.2M. It also lets 0007's workers overlap with the parent. No output change.

**0005 grid_router: dense refcount storage (`RefMap`).**
`blocked_cells` / `blocked_vias` / rung maps were `FxHashMap<u64, u16>`, and about 40 s went on hash inserts building window maps. `RefMap` is a geometrically growing `Vec<u16>` window with a hash-map overflow beyond 1<<24 cells, mirroring `BlockedBitmap`'s safety valve. A count of 0 means absent, as before. Nothing order-dependent iterates these maps. `clone_fresh` becomes a memcpy, and `is_via_blocked`'s refcount probe becomes an array load.

**0006 grid_router: Rust ports of the capsule, pad and disk rasterizers.**
Operation-for-operation ports of `_capsule_mask` + span extraction, of `pad_blocked_cells_array`'s miss path, and of `_Occ._disk_spans` (`src/raster.rs`). `_disk_spans` uses Python `**`, so the port calls the C library `pow` the way CPython's `float_pow` does (|x| for a negative base, the 1.0 short-cut), with the exponent hidden behind `black_box` so LLVM can't fold `pow(x, 2.0)` into `x*x`. The numpy/Python code stays as the fallback, and `KICAD_RUST_RASTER=0` forces it. `build_base_obstacle_map`: 83 s → 40 s (profiled).

**0007 net_rescue: speculative rescue rungs in forked workers (CLI only).**
The `_attempt_edge` rung body becomes `_rung(cfg)`, and `rescue_parallel.run_ladder` runs rungs 1.. in forked workers while the parent runs rung 0, then replays the ladder in order:
- failed with no observable state change: skip it, printing its captured stdout;
- first success: run it again in the parent, so every side effect is real;
- anything unexpected: finish serially.

"Observable state" is a structural fingerprint of the plugin's module globals (caches excluded) plus the board and window. That's why 0003 matters: it makes the build records a pure function of the rung. A rung that writes a ledger, `board_info` or a via-unblock record is never skipped, so the result is always the serial result.

Forking is only done when `threading.active_count() == 1`, so never inside KiCad. `KICAD_RESCUE_PARALLEL=0` disables it, `=N` caps the workers, and `KICAD_RESCUE_PARALLEL_DEBUG=1|2` prints stats and phase times. On my board: 132 rungs skipped, 24 winners re-run, 3 fallbacks; 212 s → 186 s. The winner re-run (14 s) and rung 0 in the parent (28 s) are what's left.

**0008 plane_region_connector: show the join number, not attempts over joins.**
The per-join line printed `[edge_idx+1/planned]`, where `edge_idx` counts every attempt (added + failed + coincident) and `planned = n_regions − 1` is the number of joins needed. A failed attempt joins nothing, so after any failure the counter ran past the total, for example `[233/231]`. The dialog's progress bar, fed `min(edge_idx+1, planned)`, also sat at 100% while work continued. It now shows the join being attempted, and adds the attempt number only when the two differ: `[230/231, attempt 233]`. Runs with no failures print exactly as before.

**0009 GUI status line: live routed/failed tallies in progress labels.**
The dialog renders progress as `label (current/total)`, and the labels carried at most a failure count, set when each net started. They now carry the tallies each phase already keeps:

| Phase | Status line |
|---|---|
| routing | `NET · 40 routed, 2 failed (43/97)` |
| diff pairs | `PAIR · 5 routed, 1 failed (6/12)` |
| reroute | `Reroute: NET · 3 re-routed, 2 failed (…)` |
| rescue | `Rescue: NET · 25 recovered, 3 improved, 3 unchanged (29/31)` |
| planes | `GND: joining plane regions · 3 failed, attempt 233 · budget x5 (230/231)` |

The plane connector also reports as each escalation stage starts (`retry-full`, `budget x5`, `narrow width`), so a long failing join no longer looks frozen. These are label changes only. Routing is unchanged, and the CLI, which passes no callback, is unaffected. Checked by driving the CLI with an injected callback that records every line exactly as the dialog would render it. The routed board stays identical.

**0010 net_rescue: speculate the bare-pad fanout-rescue calls in workers.**
Per pad, the bare-pad escape tries a dogbone tap, then a strict `generate_bga_fanout`, then a permissive one. Each runs only after the previous failed, from the same state. On my board all 74 fanout calls failed (~40 s). `rescue_parallel.speculate()` forks the strict and permissive calls before the tap. The parent uses a worker's return value only when the worker changed no observable state and the parent's state still equals the forked state; otherwise it makes the call itself.

The fab-tier record writers (`warn_fab_escalation`, `note_narrowing`) are intercepted in the worker, with every module's reference swapped. Each call is recorded with a marker in the captured stdout, and the parent makes the real calls at those markers while replaying the output. So ledger order, dedupe and printed warnings match a serial run, even when the parent logged escalations after the fork, as the dogbone tap usually does.

Workers also now disable the cyclic GC. Its first pass in a forked child copy-on-writes most of the ~2 GB heap, which had made workers much slower than the parent; this helps 0007's rung workers too.

Side by side: 155 s → 122 s (rescue pass 1: 90 s → 63 s); 72 of 74 speculations used. The board, full log and `JSON_SUMMARY` match serial.

**0011 grid_router: contiguous span stamping, tile cache, cheaper bitmap growth.**
These are code changes only, all exact; the microbench iteration count is unchanged.
- `RefMap` and `BlockedBitmap` are now column-major, so a `(gx, y_lo..y_hi)` stamping span is one contiguous run.
- `RefMap::add_span` checks the window once per span and reports 0→1 transitions to keep the S2 bitmap in sync. `BlockedBitmap::set_span` fills whole words for static stamps.
- `grow_to_include` grows geometrically and remaps only set bits. It was 2.4% of a full route in `perf`.
- `NodeStore` gets a one-entry tile cache, which skips most per-neighbour hash probes.

Microbench: 192.9 → 177 ms from the code alone. With a **PGO + `target-cpu=native`** build, trained on the microbench plus one full serial route, it reaches **125.6 ms** (−35%). Rust self-time on a full serial route: 57.6 s → 38.6 s. The routed board is identical. PGO is a build procedure, not code: instrument with `-Cprofile-generate`, train, `llvm-profdata merge`, rebuild with `-Cprofile-use`. On this CPU PGO alone gave −26% on the microbench and `target-cpu=native` about 1.5%. If you want it in your release builds, `build_router.py` would be the place.

**GPU (CUDA) was evaluated and rejected.** A* is a sequential best-first search, so a parallel GPU search expands nodes in a different order and changes routes. The rasterisers are many tiny calls: a measured CUDA round trip on an RTX 2070 is 21–53 µs against 31–90 µs per CPU call, and CUDA `pow` isn't bit-identical to glibc. The inserts are memory-bound and their results are consumed on the CPU.

## Things you'll want to decide or adjust

- **Rebasing onto HEAD.** HEAD is 893 commits past v0.22.1. Cherry-picking the series onto `04a2c68d`: 0001–0005 and 0007–0010 apply cleanly (0010 not yet re-checked on HEAD); **0006 conflicts** in `underpad.py`, because #622 now rounds the disk centre (`fx = round((x - x0)/res, 9)`). I'd keep that rounding in Python and pass the rounded `fx` and `fy` to Rust, changing the signature to take `fx, fy, r/res`, rather than replicate Python's `round(x, 9)` in Rust. A clean apply isn't the same as tested, though: `underpad.py`, `kicad_parser.py` and `net_rescue.py` have moved a lot since v0.22.1, so all of this needs re-verifying on HEAD.
- **Env knobs.** `KICAD_RUST_RASTER` and `KICAD_RESCUE_PARALLEL[_DEBUG]` are read with `os.environ` directly rather than registered in `env_knobs`. `test_653` and `test_798` pass, but you'll probably want them in the registry and the docs.
- **Versioning.** Patches 0004–0006 change the Rust crate, but I haven't bumped `Cargo.toml` / `VERSION` / `metadata.json`; I'll leave that to your release convention. Builds were done without your `Cargo.lock` (the PCM package doesn't ship one) and re-checked with it.
- **0007's fingerprint** is structural (lengths and small-container contents), so a mutation that keeps a large container's length would be missed. It caught everything on my board: `_via_unblock_failed`, `_unblock_via_sizes`, `_via_unblock_blame` and `fab_tiers._LEDGER` all triggered fallbacks as they should. But it's a heuristic guard, and the end-to-end board and log checks are the real evidence.

## Not done, possible next steps

Remaining profile (about 210 s profiled after everything): the Rust A* itself (22 s), `block_poly` / `_within` / `_point_in_poly` in the underpad escape (~15 s), `_seg_foreign_seg_dist` (~13 s), and the board-edge pass in map building (7 s). In the parallel rescue, the fanout rescue's strict-then-permissive `generate_bga_fanout` pair could reuse `run_ladder`.


## 0012–0013: exact clearance-check and fragility speed-ups

- **0012:** foreign-clearance checks validate their cache once, run their distance maths in exact Rust ports (`seg_foreign_min_dist`, `pad_foreign_min_dist`), and the two hot callers use exact "is it below the threshold?" variants that stop early.
- **0013:** `plane_fragility.publish()` rebuilds only the layers whose pours changed.

Both were checked against the original code with randomised equivalence tests (30k queries each for 0012, 5,000 steps for 0013) and give byte-identical routed boards. On a 6-layer test board the step takes 967 s instead of 1,161 s.

## 0014 (experimental, opt-in): GPU global plan

This adds a PathFinder (negotiated-congestion) engine for the #589 global planning pass, in CUDA behind the cargo feature `gpu`. It is enabled by the Options-tab checkbox, `--gpu-plan`, or `KICAD_GLOBAL_PLAN_ENGINE=gpu|auto`, and falls back to the CPU probes when no GPU is usable. With it off, routed boards are identical.

**Pre-existing bug this exposed (not caused by these patches):** on the 6-layer test board, any non-default routing order produces a few shorts. That includes your existing CPU global plan (`KICAD_GLOBAL_PLAN=1`): unmodified v0.22.1 with `KICAD_GLOBAL_PLAN=1` gives 2 `+1.8V`/`/GPIO/PD8` shorts, and its board is byte-identical to the patched plugin's (1,894 s against 878 s). In each case, the rescue or terminal-escalation stage places a through-via for a net that failed earlier (for example `Net-(Q17-C1)` at 0.45/0.2) on top of another net's inner-layer power track (`/DINOUT6`, 0.3048 mm). It reproduces with the parallel rescue disabled.

## 0015: fix for the rescue-stage shorts (root cause)

The shorts described under 0014 are fixed. The small via rung (rung 1, #568) is never stamped by `build_base_obstacle_map`, because the main route freezes its base into the static bitmap and every rung consults that bitmap. Rescue, terminal-escalation, fanout and probe maps are built with `static_base=False`, so their base copper exists only in the dynamic rung-0 map.

Once anything populates rung 1 on such a map (the rescued net's own guards do), `is_via_blocked_rung(.., 1)` stops seeing every pad and every other net's track. A boxed-endpoint rung search then places small vias on foreign copper. A probe at the centre of a foreign pad on a rescue-style map reports it as free for a small via.

**The fix:** on non-static maps, mirror the base's full-size via cells into rung 1. This is the same conservative over-block the main map gets from the static freeze. Stamping rung 1 exactly at its own via size gave byte-identical boards, so the cheap mirror loses nothing on these boards.

**Results:**
- 0 shorts in every run that previously had them.
- test board A board unchanged.

## Observation: congestion failures that the final reconciliation could close

The nets left unrouted on test board B are not boxed in:
- Each routes alone on the pads-only board.
- The 0.025 mm rescue already rules out grid quantisation.

They fail because other nets' copper fills their escape space. The final reconciliation prints "Retry with `--rip-existing-nets` …" for each one but never grants itself that authority: its `#103` harvest only reads `preexisting_blockers` events from the outer run's history. Inside a single run, nothing is pre-existing until the reconciliation sub-run, so those events never appear there.

Running exactly the hinted retry as a follow-up pass took the edited board from 23 to 19 unconnected (DRC), with 0 shorts, in 261 s. Letting the laps harvest the sub-run's own `preexisting_blockers` hints (as `#572` already does for forced links) looks like a small, contained change.

## 0016: GPU wavefront search (hybrid Lee + A*), on by default when a GPU is usable

When both 5,000-step probes of a connection fail, the full A* search (up to about 1M iterations per direction) used to take 87% of all search iterations on the test board, mostly in searches that fail. Now a GPU flood of the cost to reach the targets, using permissive legality and lower-bound costs, either proves "no path" or guides the CPU A* to the target. The CPU still makes every legality and cost decision.

**Measured (DRC unconnected, time):**

| Board | Unconnected | Time |
|---|---|---|
| test board B, edited | 23 → 19 | 1,018 → 818 s |
| test board B, old | 21 → 15 | 1,157 → 947 s |
| test board A (4 layers) | 6 → 8 | about 145 → 220 s |

- **Shadow mode:** 0 cases of "proved no path" where the CPU found one, across 345 calls.
- **Off is exact:** `--no-gpu-search` reproduces the previous boards byte for byte.
- **Prove-only mode** (`KICAD_GPU_SEARCH=prove`) leaves boards identical, but is net slower: the per-call export and flood cost about 50–90 ms. The speed gain comes from the guided search.

**Also fixed:** the dialog's GPU detection imported `grid_router` before `rust_router/` was on `sys.path`, so the GPU options were greyed out in KiCad.

## 0017: fast wide-margin segment checks

A wide track on a fine grid (+3.3V on the 0.025 mm rescue grid had a 29-cell margin) made each A* move test about 3,500 cells through `is_blocked`, so one search ran for hours, and KiCad looked hung. For margins of 3 cells or more, the search now uses a per-search bitmap cache of track blocking (64×64 tiles, word-wise scans, exact distance only for blocked cells).

- **Exact:** it returns the same answer as `segment_blocked`. 24 randomised searches gave hash-identical paths, iterations and frontiers.
- **Faster:** those searches went from 84.6 s to 6.3 s.
- **Boards unchanged:** byte-identical on both test boards.

## 0018: prune redundant leaf copper (dangling vias, plane-net continuations)

This adds a new clean-up pass at the end of every routing run, just before the final step that may not remove copper.

**Nets without pours.** It removes dead-end tracks, and vias that join nothing across layers (KiCad's `via_dangling`). `sweep_dead_ends` keeps those vias because it treats a via at a track end as an anchor.

**Plane nets.** It handles a trace run between two vias: the pin is already on the plane through its own via, and the trace carries on to a second via.
- If that second via connects nothing else, the run and the via are removed.
- If it also serves a pin, only the run is removed.
- Stitching vias and a pin's own via are never touched.

**Safety:** each removal must leave the zone-aware connectivity no worse. Unfinished nets are skipped.

**Measured:** KiCad `via_dangling` went from 6 to 2 on test board A and from 8 to 4 on the test board B plan, with unconnected items no worse (test board A improved from 8 to 7) and 0 shorts.

## 0019–0021

- **0019, clean-up extension:**
  - Junk copper is removed on any net: duplicate vias and segments, slivers inside a via, and track ends aimed at a pin on another layer.
  - Pin attachment is now layer-aware.
  - The pass is about 3× faster (390 s → 134 s per 6-layer step), with an identical KiCad DRC result.
- **0020, Board Setup via minimums:** these now count as the fab's via capability, so net-class 0.3/0.15 vias are honoured under the default `auto` tier instead of being raised to 0.45/0.2, and never escalated below the declared size. Boards that don't declare smaller minimums are unchanged.
- **0021, opt-in GPU rip-up planning** (`--ripup-blocker-select gpu`):
  - It finds the cheapest crossing set over all rippable nets, then tests alternative sets for re-routability.
  - It was neutral on the 6-layer test step.
  - **Note for you:** `mincut_probe_order` soft-costs only `blocked_cells`. Since #815, segment keep-outs live in `blocked_cell_spans`, so the CPU min-cut probe misses most track copper.

## 0022 — Honour net-class clearances for GND return vias and headless plan runs

On a 4-layer board with 50R (0.508 mm) and 1A (0.1524 mm) classes, a plan that
routed to 100% still added 47 DRC errors; 44 were GND return vias. Causes:

1. **Headless plan runs passed an empty class map.** With no live pcbnew board,
   `_get_net_classes_from_board()` returns only `Default`, so the route and plane
   tabs built `net_clearances = {}`, and an explicit empty map also stops the
   engine's own `.kicad_pro` auto-read. Fix: fall back to
   `list_nets.net_clearance_map_by_id` (`fanout_gui._net_clearances_from_project`).
2. **The GND-via placer ignored classes.** Both callers (plane tab and
   `route_planes` main) built its obstacle map without `net_clearances`, and
   via-to-via spacing used the run clearance only (and skipped the signal via
   entirely). Fix: install the class map, and use a per-existing-via pairwise
   minimum, including the signal via.
3. **Grid quantisation.** The placer's check is cell-based; at the plan's 0.1 mm
   grid it passed vias up to about 0.03 mm inside clearance. Fix: re-measure each
   candidate exactly against foreign segments and pads
   (`check_drc.point_to_pad_distance`) at max(run, class, pad-local).

Result: 100% routed, 0 routing-added DRC errors (previously 47), with the same
150 GND vias placed.

## 0023 — "Fix DRC settings after routing" opt-in (local preference)

The post-route fix lowered the project's class clearances (50R 0.508 → 0.127) and
`min_hole_clearance` (0.2 → 0.127) to the routed values, which hides real
violations. It is now off by default in the dialog and CLIs; `--fix-drc-settings`
opts in. This one may not suit upstream as-is; it's offered as a discussion point.

## 0024 — Diff pairs: pitch at the drawn width; diff search weight 1.3; turn cost 3000

- **Pitch bug:** the pair pitch used the nominal `--track-width` (0.127) while the
  impedance solve drew 0.166 mm on F.Cu. The real gap became 0.091 mm (below the
  0.127 clearance), so the #318 neck narrowed D+ to 0.0996 mm: the two members
  ended up at different widths. The pitch now uses the widest layer width.
- `DIFF_PAIR_HEURISTIC_WEIGHT = 1.3`: at 2.3 a USB pair zig-zagged for 77 mm
  through a pad channel; at 1.3 it runs straight for 63 mm, in the same time.
- `TURN_COST` 3000. Single-ended `HEURISTIC_WEIGHT` stays 2.3: combined with turn
  cost 3000, a weight of 1.3 left test board A at 11 unconnected (was 7); each change
  alone improved it (6 and 3).

## 0025 — Clean-up: shadow tracks

A track whose copper lies inside other same-net copper is removed, as is a long
(≥ 1 mm) parallel overlap of an equal-or-wider track. Both removals are graded for
connectivity. Found on a USB-C D+ A6/B6 join that ran 40 mm alongside the pair.

## 0026 — Opt-in growth of vias in small pads

`pad_via_upsize.py`, the dialog option and the plan parameter `upsize_pad_vias`.
It works on pads up to 3 mm: it grows the via diameter while it stays inside the
pad, then the drill down to the minimum annular ring. All checks use KiCad shapes.

## 0027 — Weld stranded pour islands with a via (default on)

When KiCad reports unconnected items, the pass looks for zone islands that hold a
pad of their own net but no via or through-hole pad of that net. Each such island
gets a via (the net's usual via, falling back to Board Setup's smallest), checked
with KiCad shapes. A via is kept only if KiCad's unconnected count drops after a
refill. This fixes a GND pad boxed in by signal tracks, which the exact-fill
oracle detects but cannot route on its own layer.

## 0028 — Replace small-drill vias by re-routing (opt-in); welder stub escapes

`small_via_eliminate.py`, the dialog option "Replace small-drill vias by re-routing
(slow)" and the plan param `eliminate_small_vias`. For each smallest-drill via it
rips the blocking tracks and the via's signal net, then re-routes them with small
vias forbidden (Board Setup minimum raised, fab tier pinned to standard).

- Plane nets are never routed by this pass. Their pads are reconnected by the
  welder with full-size vias.
- Pads that a previous attempt caged get escapes reserved before the next attempt.
- A result is kept only if KiCad DRC does not get worse.

The welder gains a pad → track → via fallback and an optional minimum via size.
`nudge_for_vias.py` is an experimental placement-search CLI. On test board B the
number of small-drill vias went from 5 to between 0 and 1, with no DRC errors added.

## 0029 — Small-via replacement on by default (local preference)

The dialog option now defaults to on. It returns immediately on a board with no smallest-drill vias. Because plans reset the dialog for each step, a multi-step plan should set `eliminate_small_vias: false` on every route step except the last.

## 0030 — Board Setup minimum via as a hard floor; repair boxed-in plane pads

- `fab_tiers`: Board Setup's minimum via is now the via floor at every
  escalation rung, in both directions. Patch 0020 only lowered rungs, so with
  Board Setup at 0.45/0.3 the `auto` ladder still placed 0.45/0.20 and 0.30/0.15
  vias. Under every policy, a stale minimum (one below the run's own via request)
  is dropped.
- `small_via_eliminate`: it now also repairs plane-net pads that signal tracks
  have boxed in. It rips the cheapest blocking signal nets, reserves a stub and a
  full-size via, and re-routes. It also runs when KiCad reports unconnected items,
  and it grades with the project's rules.

Measured on test board B with a 0.45/0.3 minimum: 628 vias, none with a drill
below 0.3 mm, 0 unconnected, and no DRC errors added.

## 0031 — Sweep dangling copper that KiCad's DRC reports

The plugin's clean-up checks plane-net copper against its own fill model. Where
KiCad's exact fill differs, it can keep vias and stubs that KiCad reports as
dangling. The small-via pass now ends by running DRC, removing the reported
dangling items, and re-checking. Each round is kept only if unconnected items and
errors do not rise.

## 0032 — Widen power-net necks after routing

`widen_necks.py` widens each narrowed power-net segment to the largest width that
fits. The width never goes past the net's power width, and never wider than the
smaller side of a pad it lands on. Clearance and hole clearance are checked with
KiCad shapes. The small-via pass runs it per net and keeps a net only if DRC does
not get worse; a failing net is bisected so the passing parts are kept.

Measured: 210 of 230 necks widened, with DRC unchanged.

## 0033 — Honour via-in-pad off everywhere; exposed pads exempt

- The headless plan runner loads the user's KiCad site policies, so plan runs
  behave as they do inside KiCad.
- When via-in-pad is off, these all keep vias at the same-net pad keep-off: GND
  return vias, the island welder, the escape reservation, in-pad via growth and
  the small-via pass.
- A part's large exposed centre pad still takes thermal vias.

Measured: keep-off violations went from 17 to 1, and 42 thermal vias were
restored to exposed pads.

🤖 Generated with [Claude Code](https://claude.com/claude-code)
