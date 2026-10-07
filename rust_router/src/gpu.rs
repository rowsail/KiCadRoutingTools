//! GPU negotiated-congestion (PathFinder) global router.
//!
//! Routes every net on a W x H x L grid. Iteration 1 routes all nets at once
//! (every net's current leg searched in one batched wavefront on the GPU).
//! Each net's route stamps a width-aware keep-out halo into a shared
//! occupancy grid; overlap raises present + history costs. Later iterations
//! reroute only the nets touching overused cells, in a few shuffled batches,
//! removing each batch's old occupancy and stamping its new routes before the
//! next batch -- so nets see each other's moves (no lock-step oscillation)
//! while each batch still searches many nets in parallel. Stops when no cell
//! is shared or after max_iters.
//!
//! The result is a ROUGH plan (one path per net, legs concatenated) for
//! global_plan.py's consumers; the CPU detailed router stays the authority on
//! exact geometry and DRC.
//!
//! Kernels: kernels/pathfinder.cu, compiled to PTX by build.rs (nvcc) and
//! JIT-loaded by the driver, so the module still imports on a machine without
//! a GPU (gpu_available() == False there).

use std::sync::Arc;
use std::time::Instant;

use cudarc::driver::{CudaContext, CudaFunction, CudaModule, CudaSlice, CudaStream,
                     DeviceRepr, LaunchConfig, PushKernelArg};
use cudarc::nvrtc::Ptx;
use numpy::{PyReadonlyArray2, PyReadonlyArray3};
use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3::types::PyDict;

const PTX: &str = include_str!(concat!(env!("OUT_DIR"), "/pathfinder.ptx"));
const INF64: u64 = u64::MAX;

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct Slot {
    x0: i32,
    y0: i32,
    ww: i32,
    wh: i32,
    off: i64,
    boff: i64,
}
unsafe impl DeviceRepr for Slot {}

fn derr(e: impl std::fmt::Debug) -> PyErr {
    PyRuntimeError::new_err(format!("CUDA: {e:?}"))
}

fn cfg(n: usize) -> LaunchConfig {
    LaunchConfig::for_num_elems(n.max(1) as u32)
}

fn per_slot_cfg(n_slots: usize) -> LaunchConfig {
    LaunchConfig { grid_dim: (n_slots.max(1) as u32, 1, 1), block_dim: (256, 1, 1), shared_mem_bytes: 0 }
}

/// True when a CUDA device is usable.
#[pyfunction]
pub fn gpu_available() -> bool {
    CudaContext::new(0).is_ok()
}

struct Kernels {
    relax: CudaFunction,
    fill_u64: CudaFunction,
    fill_u32: CudaFunction,
    seed: CudaFunction,
    set_bits: CudaFunction,
    gather_dist: CudaFunction,
    backtrack: CudaFunction,
    stamp_occ: CudaFunction,
    unstamp: CudaFunction,
    flag_congested: CudaFunction,
    update_hist: CudaFunction,
    split_far: CudaFunction,
    set_counters: CudaFunction,
}

/// One net's routing state (window-local cell indices).
#[derive(Clone)]
struct NetState {
    tree: Vec<u32>,
    remaining: Vec<Vec<u32>>,
    legs: Vec<Vec<u32>>,
    failed: bool,
}

/// Per-call device buffers.
struct Bufs {
    slots: Vec<Slot>,
    d_slots: CudaSlice<Slot>,
    dist: CudaSlice<u64>,
    stamp: CudaSlice<u32>,
    own: CudaSlice<u32>,
    zero_bits: CudaSlice<u32>,
    tgt: CudaSlice<u32>,
    near_a: CudaSlice<u64>,
    near_b: CudaSlice<u64>,
    far_a: CudaSlice<u64>,
    far_b: CudaSlice<u64>,
    far_cap: u32,
    counters: CudaSlice<u32>,   // [n_near, n_far, n_far_out, min_far]
    best_tgt: CudaSlice<u32>,
    round: u32,
    rounds_total: u64,
    far_overflow: bool,
}

#[pyclass(unsendable)]
pub struct GpuGlobalRouter {
    stream: Arc<CudaStream>,
    _module: Arc<CudaModule>,
    k: Kernels,
    w: i32,
    h: i32,
    l: i32,
    blocked: CudaSlice<u8>,
    via_blocked: CudaSlice<u8>,
    hist: CudaSlice<f32>,
    occ: CudaSlice<u32>,
    layer_cost: CudaSlice<i32>,
    pin_mask: CudaSlice<u8>,
    via_cost: i32,
}

impl GpuGlobalRouter {
    /// Route the given nets (fresh trees) leg by leg, all of them in each
    /// batched wavefront, against the current occ / hist. Their own
    /// occupancy must already be removed (unstamp) by the caller.
    #[allow(clippy::too_many_arguments)]
    fn route_batch(&mut self, b: &mut Bufs, states: &mut [NetState], init: &[NetState],
                   batch: &[usize], pres: f32, big_net_pins: usize, max_rounds: u32,
                   block_pen: f32, delta: u32) -> PyResult<()> {
        let st = self.stream.clone();
        let (w, h, l) = (self.w, self.h, self.l);
        for &i in batch { states[i] = init[i].clone(); }
        loop {
            let active: Vec<usize> = batch.iter().copied()
                .filter(|&i| !states[i].failed && !states[i].remaining.is_empty()
                        && !states[i].tree.is_empty())
                .collect();
            if active.is_empty() { break; }
            for &i in &active {
                let s = b.slots[i];
                let n = s.ww as i64 * s.wh as i64 * l as i64;
                let mut v = b.dist.slice_mut(s.off as usize..(s.off + n) as usize);
                unsafe { st.launch_builder(&self.k.fill_u64).arg(&mut v).arg(&n).arg(&INF64)
                    .launch(cfg(n as usize)) }.map_err(derr)?;
                let nw = (n + 31) / 32;
                let z: u32 = 0;
                let mut tv = b.tgt.slice_mut(s.boff as usize..(s.boff + nw) as usize);
                unsafe { st.launch_builder(&self.k.fill_u32).arg(&mut tv).arg(&nw).arg(&z)
                    .launch(cfg(nw as usize)) }.map_err(derr)?;
            }
            let mut tgt_items: Vec<u64> = Vec::new();
            let mut seeds: Vec<u64> = Vec::new();
            for &i in &active {
                for term in &states[i].remaining { for &c in term {
                    tgt_items.push(((i as u64) << 32) | c as u64);
                } }
                for &c in &states[i].tree { seeds.push(((i as u64) << 32) | c as u64); }
            }
            if !tgt_items.is_empty() {
                let d_t = st.clone_htod(&tgt_items).map_err(derr)?;
                let n = tgt_items.len() as u32;
                unsafe { st.launch_builder(&self.k.set_bits).arg(&d_t).arg(&n).arg(&mut b.tgt)
                    .arg(&b.d_slots).launch(cfg(n as usize)) }.map_err(derr)?;
            }
            b.round = b.round.wrapping_add(1).max(1);
            let d_seed = st.clone_htod(&seeds).map_err(derr)?;
            let ns = seeds.len() as u32;
            unsafe { st.launch_builder(&self.k.seed).arg(&d_seed).arg(&ns).arg(&mut b.dist)
                .arg(&b.d_slots).arg(&mut b.stamp).arg(&b.round).launch(cfg(ns as usize)) }
                .map_err(derr)?;
            // best target cost per slot: "not reached"
            let nslots = b.slots.len() as i64;
            let umax = u32::MAX;
            unsafe { st.launch_builder(&self.k.fill_u32).arg(&mut b.best_tgt).arg(&nslots).arg(&umax)
                .launch(cfg(nslots as usize)) }.map_err(derr)?;
            st.memcpy_htod(&seeds, &mut b.near_a.slice_mut(0..seeds.len())).map_err(derr)?;
            // Near-far wavefront: relax everything cheaper than T; push the
            // rest to the far queue; when near drains, raise T and move far
            // items that fell under it back to near.
            let mut n_near = ns;
            let mut n_far: u32 = 0;
            let mut t_thr: u32 = delta;
            let mut rounds = 0u32;
            loop {
                while n_near > 0 && rounds < max_rounds {
                    b.round = b.round.wrapping_add(1).max(1);
                    rounds += 1;
                    let (z, m) = (0u32, u32::MAX);
                    unsafe { st.launch_builder(&self.k.set_counters).arg(&mut b.counters)
                        .arg(&z).arg(&n_far).arg(&z).arg(&m).launch(cfg(1)) }.map_err(derr)?;
                    unsafe { st.launch_builder(&self.k.relax)
                        .arg(&b.near_a).arg(&n_near).arg(&mut b.near_b).arg(&mut b.far_a)
                        .arg(&mut b.counters).arg(&b.far_cap)
                        .arg(&mut b.stamp).arg(&b.round).arg(&t_thr).arg(&mut b.dist).arg(&b.d_slots)
                        .arg(&self.blocked).arg(&self.via_blocked).arg(&self.hist)
                        .arg(&self.occ).arg(&b.tgt).arg(&mut b.best_tgt).arg(&self.layer_cost)
                        .arg(&w).arg(&h).arg(&l).arg(&self.via_cost).arg(&pres).arg(&block_pen)
                        .launch(cfg(n_near as usize)) }.map_err(derr)?;
                    let c = st.clone_dtoh(&b.counters).map_err(derr)?;
                    n_near = c[0];
                    n_far = c[1];
                    if n_far > b.far_cap { b.far_overflow = true; n_far = b.far_cap; }
                    std::mem::swap(&mut b.near_a, &mut b.near_b);
                }
                if n_far == 0 || rounds >= max_rounds { break; }
                // raise the threshold (step grows with T, so high-congestion
                // cost ranges take few steps); skip empty ranges via the far min
                t_thr = t_thr.saturating_add(delta.max(t_thr >> 4));
                let mut moved = false;
                for _pass in 0..2 {
                    b.round = b.round.wrapping_add(1).max(1);
                    let (z, m) = (0u32, u32::MAX);
                    unsafe { st.launch_builder(&self.k.set_counters).arg(&mut b.counters)
                        .arg(&z).arg(&z).arg(&z).arg(&m).launch(cfg(1)) }.map_err(derr)?;
                    unsafe { st.launch_builder(&self.k.split_far)
                        .arg(&b.far_a).arg(&n_far).arg(&mut b.near_a).arg(&mut b.far_b)
                        .arg(&mut b.counters)
                        .arg(&mut b.stamp).arg(&b.round).arg(&t_thr).arg(&b.dist).arg(&b.d_slots)
                        .arg(&b.best_tgt)
                        .launch(cfg(n_far as usize)) }.map_err(derr)?;
                    let c = st.clone_dtoh(&b.counters).map_err(derr)?;
                    n_near = c[0];
                    n_far = c[2];
                    let min_far = c[3];
                    std::mem::swap(&mut b.far_a, &mut b.far_b);
                    if n_near > 0 || n_far == 0 { moved = n_near > 0; break; }
                    t_thr = min_far.saturating_add(delta.max(min_far >> 4)); // jump to cheapest far item
                }
                if !moved && n_far == 0 { break; }
            }
            b.rounds_total += rounds as u64;
            // pick the next terminal(s) per net
            let mut gather: Vec<u64> = Vec::new();
            let mut gmap: Vec<(usize, usize)> = Vec::new();
            for &i in &active {
                for (ti, term) in states[i].remaining.iter().enumerate() {
                    for &c in term { gather.push(((i as u64) << 32) | c as u64); gmap.push((i, ti)); }
                }
            }
            let d_g = st.clone_htod(&gather).map_err(derr)?;
            let mut d_out = st.alloc_zeros::<u64>(gather.len().max(1)).map_err(derr)?;
            let ng = gather.len() as u32;
            unsafe { st.launch_builder(&self.k.gather_dist).arg(&d_g).arg(&ng).arg(&b.dist)
                .arg(&b.d_slots).arg(&mut d_out).launch(cfg(ng as usize)) }.map_err(derr)?;
            let got = st.clone_dtoh(&d_out).map_err(derr)?;
            let mut best: std::collections::BTreeMap<(usize, usize), (u64, u32)> = Default::default();
            for (k, &(i, ti)) in gmap.iter().enumerate() {
                let v = got[k];
                if v == INF64 { continue; }
                let cell = (gather[k] & 0xFFFF_FFFF) as u32;
                let e = best.entry((i, ti)).or_insert((v, cell));
                if v < e.0 { *e = (v, cell); }
            }
            let mut starts: Vec<u64> = Vec::new();
            let mut start_of: Vec<(usize, usize)> = Vec::new();
            for &i in &active {
                let mut cands: Vec<(u64, usize, u32)> = best.range((i, 0)..(i + 1, 0))
                    .map(|(&(_, ti), &(v, c))| (v >> 32, ti, c)).collect();
                if cands.is_empty() { states[i].failed = true; continue; }
                cands.sort();
                let rem = states[i].remaining.len();
                let take = if rem + 1 > big_net_pins { (rem / 4).max(1) } else { 1 };
                for &(_, ti, c) in cands.iter().take(take) {
                    starts.push(((i as u64) << 32) | c as u64);
                    start_of.push((i, ti));
                }
            }
            if starts.is_empty() { continue; }
            let max_len: u32 = 1 << 16;
            let d_s = st.clone_htod(&starts).map_err(derr)?;
            let mut d_path = st.alloc_zeros::<u32>(starts.len() * max_len as usize).map_err(derr)?;
            let mut d_len = st.alloc_zeros::<u32>(starts.len()).map_err(derr)?;
            let nst = starts.len() as u32;
            unsafe { st.launch_builder(&self.k.backtrack).arg(&d_s).arg(&nst).arg(&b.dist)
                .arg(&b.d_slots).arg(&mut d_path).arg(&mut d_len).arg(&max_len)
                .launch(cfg(nst as usize)) }.map_err(derr)?;
            let lens = st.clone_dtoh(&d_len).map_err(derr)?;
            let paths = st.clone_dtoh(&d_path).map_err(derr)?;
            let mut done: Vec<(usize, usize)> = Vec::new();
            for (k, &(i, ti)) in start_of.iter().enumerate() {
                let n = lens[k] as usize;
                let base = k * max_len as usize;
                let mut p: Vec<u32> = paths[base..base + n].to_vec();
                p.reverse(); // tree -> terminal
                states[i].tree.extend_from_slice(&p);
                states[i].legs.push(p);
                done.push((i, ti));
            }
            done.sort_by(|a, b| b.cmp(a));
            done.dedup();
            for (i, ti) in done { states[i].remaining.remove(ti); }
        }
        Ok(())
    }

    /// Stamp the batch's routes into occ and their own-bits.
    fn stamp_batch(&mut self, b: &mut Bufs, states: &[NetState], batch: &[usize],
                   radii: &[(i32, i32)]) -> PyResult<()> {
        let st = self.stream.clone();
        let (w, h, l) = (self.w, self.h, self.l);
        let mut items: Vec<u64> = Vec::new();
        let mut radius: Vec<i32> = Vec::new();
        let mut is_via: Vec<u8> = Vec::new();
        for &i in batch {
            let s = &b.slots[i];
            let plane = s.ww as u32 * s.wh as u32;
            let (tr, vr) = radii[i];
            for leg in &states[i].legs {
                for (j, &c) in leg.iter().enumerate() {
                    let via = (j > 0 && leg[j - 1] % plane == c % plane)
                        || (j + 1 < leg.len() && leg[j + 1] % plane == c % plane);
                    items.push(((i as u64) << 32) | c as u64);
                    radius.push(if via { vr } else { tr });
                    is_via.push(via as u8);
                }
            }
        }
        if items.is_empty() { return Ok(()); }
        let d_i = st.clone_htod(&items).map_err(derr)?;
        let d_r = st.clone_htod(&radius).map_err(derr)?;
        let d_v = st.clone_htod(&is_via).map_err(derr)?;
        let n = items.len() as u32;
        unsafe { st.launch_builder(&self.k.stamp_occ).arg(&d_i).arg(&d_r).arg(&d_v).arg(&n)
            .arg(&mut b.own).arg(&mut self.occ).arg(&b.d_slots).arg(&self.pin_mask)
            .arg(&w).arg(&h).arg(&l)
            .launch(cfg(n as usize)) }.map_err(derr)?;
        Ok(())
    }

    fn unstamp_batch(&mut self, b: &mut Bufs, batch: &[usize]) -> PyResult<()> {
        if batch.is_empty() { return Ok(()); }
        let st = self.stream.clone();
        let (w, h, l) = (self.w, self.h, self.l);
        let list: Vec<u32> = batch.iter().map(|&i| i as u32).collect();
        let d_l = st.clone_htod(&list).map_err(derr)?;
        unsafe { st.launch_builder(&self.k.unstamp).arg(&d_l).arg(&mut b.own).arg(&mut self.occ)
            .arg(&b.d_slots).arg(&w).arg(&h).arg(&l).launch(per_slot_cfg(list.len())) }
            .map_err(derr)?;
        Ok(())
    }
}

#[pymethods]
impl GpuGlobalRouter {
    /// blocked: [L, H, W] u8 track keep-outs; via_blocked: [H, W] u8;
    /// layer_costs: per layer (1000 = 1.0x); via_cost in the same units.
    #[new]
    #[pyo3(signature = (blocked, via_blocked, layer_costs, via_cost, pin_mask=None))]
    fn new(blocked: PyReadonlyArray3<u8>, via_blocked: PyReadonlyArray2<u8>,
           layer_costs: Vec<i32>, via_cost: i32,
           pin_mask: Option<PyReadonlyArray3<u8>>) -> PyResult<Self> {
        let bl = blocked.as_array();
        let (l, h, w) = (bl.shape()[0], bl.shape()[1], bl.shape()[2]);
        let vb = via_blocked.as_array();
        if vb.shape() != [h, w] || layer_costs.len() != l {
            return Err(PyRuntimeError::new_err("grid shape mismatch"));
        }
        let ctx = CudaContext::new(0).map_err(derr)?;
        let stream = ctx.default_stream();
        let module = ctx.load_module(Ptx::from_src(PTX)).map_err(derr)?;
        let f = |n: &str| module.load_function(n).map_err(derr);
        let k = Kernels {
            relax: f("relax")?, fill_u64: f("fill_u64")?, fill_u32: f("fill_u32")?,
            seed: f("seed")?, set_bits: f("set_bits")?, gather_dist: f("gather_dist")?,
            backtrack: f("backtrack")?, stamp_occ: f("stamp_occ")?, unstamp: f("unstamp")?,
            flag_congested: f("flag_congested")?, update_hist: f("update_hist")?,
            split_far: f("split_far")?, set_counters: f("set_counters")?,
        };
        let bvec: Vec<u8> = bl.iter().copied().collect();
        let vvec: Vec<u8> = vb.iter().copied().collect();
        let n = l * h * w;
        let pm: Vec<u8> = match &pin_mask {
            Some(a) => {
                let a = a.as_array();
                if a.shape() != [l, h, w] { return Err(PyRuntimeError::new_err("pin_mask shape")); }
                a.iter().copied().collect()
            }
            None => vec![0u8; n],
        };
        Ok(Self {
            pin_mask: stream.clone_htod(&pm).map_err(derr)?,
            blocked: stream.clone_htod(&bvec).map_err(derr)?,
            via_blocked: stream.clone_htod(&vvec).map_err(derr)?,
            hist: stream.alloc_zeros::<f32>(n).map_err(derr)?,
            occ: stream.alloc_zeros::<u32>(n).map_err(derr)?,
            layer_cost: stream.clone_htod(&layer_costs).map_err(derr)?,
            stream, _module: module, k,
            w: w as i32, h: h as i32, l: l as i32, via_cost,
        })
    }

    /// Route all nets with negotiated congestion.
    ///
    /// nets: [(net_id, terminals, track_radius, via_radius)], terminals =
    /// [[(x, y, layer), ...] per pad] in this router's grid coords.
    /// Returns ({net_id: [(x, y, layer), ...]} -- legs concatenated; a leg
    /// boundary is where consecutive points are not neighbours -- , stats).
    #[pyo3(signature = (nets, max_iters=40, margin=20, pres0=0.5, pres_mult=1.6,
                        hfac=0.4, big_net_pins=8, max_rounds=200000, batches_per_iter=4,
                        seed=12345, block_pen=20.0, delta=3000, pres_max=200.0, stall_iters=6))]
    #[allow(clippy::too_many_arguments)]
    fn route<'py>(&mut self, py: Python<'py>,
                  nets: Vec<(i64, Vec<Vec<(i32, i32, i32)>>, i32, i32)>,
                  max_iters: u32, margin: i32, pres0: f32, pres_mult: f32,
                  hfac: f32, big_net_pins: usize, max_rounds: u32,
                  batches_per_iter: usize, seed: u64, block_pen: f32, delta: u32,
                  pres_max: f32, stall_iters: u32)
                  -> PyResult<(Bound<'py, PyDict>, Bound<'py, PyDict>)> {
        let t0 = Instant::now();
        let (w, h, l) = (self.w, self.h, self.l);
        let mut slots: Vec<Slot> = Vec::with_capacity(nets.len());
        let (mut off, mut boff) = (0i64, 0i64);
        for (_, terms, _, _) in &nets {
            let (mut x0, mut y0, mut x1, mut y1) = (i32::MAX, i32::MAX, i32::MIN, i32::MIN);
            for t in terms { for &(x, y, _) in t {
                x0 = x0.min(x); y0 = y0.min(y); x1 = x1.max(x); y1 = y1.max(y);
            } }
            if x0 > x1 { x0 = 0; y0 = 0; x1 = 0; y1 = 0; }
            let x0 = (x0 - margin).max(0); let y0 = (y0 - margin).max(0);
            let x1 = (x1 + margin).min(w - 1); let y1 = (y1 + margin).min(h - 1);
            let (ww, wh) = (x1 - x0 + 1, y1 - y0 + 1);
            let cells = ww as i64 * wh as i64 * l as i64;
            slots.push(Slot { x0, y0, ww, wh, off, boff });
            off += cells;
            boff += (cells + 31) / 32;
        }
        let total = off as usize;
        let words = (boff as usize).max(1);
        let st = self.stream.clone();
        let far_cap: u32 = total.max(1024).min(u32::MAX as usize / 2) as u32;
        let mut b = Bufs {
            d_slots: st.clone_htod(&slots).map_err(derr)?,
            dist: st.alloc_zeros::<u64>(total.max(1)).map_err(derr)?,
            stamp: st.alloc_zeros::<u32>(total.max(1)).map_err(derr)?,
            own: st.alloc_zeros::<u32>(words).map_err(derr)?,
            zero_bits: st.alloc_zeros::<u32>(words).map_err(derr)?,
            tgt: st.alloc_zeros::<u32>(words).map_err(derr)?,
            near_a: st.alloc_zeros::<u64>(total.max(1)).map_err(derr)?,
            near_b: st.alloc_zeros::<u64>(total.max(1)).map_err(derr)?,
            far_a: st.alloc_zeros::<u64>(far_cap as usize).map_err(derr)?,
            far_b: st.alloc_zeros::<u64>(far_cap as usize).map_err(derr)?,
            far_cap,
            counters: st.alloc_zeros::<u32>(4).map_err(derr)?,
            best_tgt: st.alloc_zeros::<u32>(nets.len().max(1)).map_err(derr)?,
            slots, round: 0, rounds_total: 0, far_overflow: false,
        };
        // fresh occupancy / history for this call
        let n_cells = (l * h * w) as i64;
        let z: u32 = 0;
        unsafe { st.launch_builder(&self.k.fill_u32).arg(&mut self.occ).arg(&n_cells).arg(&z)
            .launch(cfg(n_cells as usize)) }.map_err(derr)?;
        let hist0 = vec![0f32; n_cells as usize];
        st.memcpy_htod(&hist0, &mut self.hist).map_err(derr)?;

        let local = |s: &Slot, x: i32, y: i32, ly: i32| -> Option<u32> {
            let (lx, lyy) = (x - s.x0, y - s.y0);
            if lx < 0 || lyy < 0 || lx >= s.ww || lyy >= s.wh || ly < 0 || ly >= l { return None; }
            Some(((ly as u32 * s.wh as u32) + lyy as u32) * s.ww as u32 + lx as u32)
        };
        let init: Vec<NetState> = nets.iter().zip(&b.slots).map(|((_, terms, _, _), s)| {
            let mut tl: Vec<Vec<u32>> = terms.iter().map(|t| {
                t.iter().filter_map(|&(x, y, ly)| local(s, x, y, ly)).collect::<Vec<u32>>()
            }).filter(|v| !v.is_empty()).collect();
            let first = if tl.is_empty() { Vec::new() } else { tl.remove(0) };
            NetState { tree: first, remaining: tl, legs: Vec::new(), failed: false }
        }).collect();
        let radii: Vec<(i32, i32)> = nets.iter().map(|n| (n.2, n.3)).collect();
        let mut states = init.clone();
        let all: Vec<usize> = (0..nets.len()).collect();

        // iteration 1: everyone at once
        let mut pres = pres0;
        self.route_batch(&mut b, &mut states, &init, &all, pres, big_net_pins, max_rounds,
                         block_pen, delta)?;
        self.stamp_batch(&mut b, &states, &all, &radii)?;
        let mut rng = seed | 1;
        let mut overused = 0u32;
        let mut iters = 1u32;
        let mut rerouted_total = 0usize;
        let mut best_over = u32::MAX;
        let mut since_best = 0u32;
        let mut best_states = states.clone();
        let mut over_hist: Vec<u32> = Vec::new();
        loop {
            // overuse count + history
            let mut d_over = st.alloc_zeros::<u32>(1).map_err(derr)?;
            unsafe { st.launch_builder(&self.k.update_hist).arg(&self.occ).arg(&mut self.hist)
                .arg(&n_cells).arg(&hfac).arg(&mut d_over).launch(cfg(n_cells as usize)) }
                .map_err(derr)?;
            overused = st.clone_dtoh(&d_over).map_err(derr)?[0];
            over_hist.push(overused);
            // keep the least-congested solution seen; stop when it stalls
            if overused < best_over {
                best_over = overused; since_best = 0; best_states = states.clone();
            } else {
                since_best += 1;
            }
            if overused == 0 || iters >= max_iters || since_best >= stall_iters { break; }
            iters += 1;
            pres = (pres * pres_mult).min(pres_max);
            // congested nets
            let mut d_flag = st.alloc_zeros::<u32>(nets.len().max(1)).map_err(derr)?;
            unsafe { st.launch_builder(&self.k.flag_congested).arg(&b.own).arg(&self.occ)
                .arg(&b.d_slots).arg(&mut d_flag).arg(&w).arg(&h).arg(&l)
                .launch(per_slot_cfg(nets.len())) }.map_err(derr)?;
            let flags = st.clone_dtoh(&d_flag).map_err(derr)?;
            let mut cong: Vec<usize> = (0..nets.len()).filter(|&i| flags[i] != 0).collect();
            if cong.is_empty() { break; }
            // deterministic shuffle (xorshift)
            for i in (1..cong.len()).rev() {
                rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17;
                cong.swap(i, (rng % (i as u64 + 1)) as usize);
            }
            let nb = batches_per_iter.max(1).min(cong.len());
            let chunk = cong.len().div_ceil(nb);
            for batch in cong.chunks(chunk) {
                self.unstamp_batch(&mut b, batch)?;
                self.route_batch(&mut b, &mut states, &init, batch, pres, big_net_pins, max_rounds,
                                 block_pen, delta)?;
                self.stamp_batch(&mut b, &states, batch, &radii)?;
                rerouted_total += batch.len();
            }
        }
        st.synchronize().map_err(derr)?;
        let states = best_states;
        let overused = best_over.min(overused);

        let out = PyDict::new(py);
        let mut n_failed = 0usize;
        for (i, ((nid, _, _, _), s)) in nets.iter().zip(&b.slots).enumerate() {
            let ns = &states[i];
            if ns.failed || !ns.remaining.is_empty() { n_failed += 1; }
            let mut pts: Vec<(i32, i32, i32)> = Vec::new();
            for leg in &ns.legs {
                for &c in leg {
                    let lx = (c % s.ww as u32) as i32;
                    let t = c / s.ww as u32;
                    let ly = (t % s.wh as u32) as i32;
                    let layer = (t / s.wh as u32) as i32;
                    pts.push((lx + s.x0, ly + s.y0, layer));
                }
            }
            if !pts.is_empty() { out.set_item(*nid, pts)?; }
        }
        let stats = PyDict::new(py);
        stats.set_item("iterations", iters)?;
        stats.set_item("overused_cells", overused)?;
        stats.set_item("failed_nets", n_failed)?;
        stats.set_item("rerouted", rerouted_total)?;
        stats.set_item("wavefront_rounds", b.rounds_total)?;
        stats.set_item("window_cells", total)?;
        stats.set_item("far_overflow", b.far_overflow)?;
        stats.set_item("overuse_by_iter", over_hist)?;
        stats.set_item("seconds", t0.elapsed().as_secs_f64())?;
        Ok((out, stats))
    }
}
