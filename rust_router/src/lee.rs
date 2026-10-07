//! GPU wavefront flood (kernels/lee.cu) for the hybrid search
//! (GridRouter.route_with_frontier_gpu).
//!
//! One process-wide engine, created on first use and reused (CUDA context,
//! module, device buffers grown as needed). It refuses to run in any process
//! other than the one that created it: the parallel rescue forks workers,
//! and a forked child cannot use its parent's CUDA context -- callers then
//! fall back to the CPU search.

use std::sync::{Arc, Mutex, OnceLock};

use cudarc::driver::{CudaContext, CudaFunction, CudaModule, CudaSlice, CudaStream,
                     LaunchConfig, PushKernelArg};
use cudarc::nvrtc::Ptx;

const PTX: &str = include_str!(concat!(env!("OUT_DIR"), "/lee.ptx"));
pub const INF: u32 = u32::MAX;

struct Engine {
    pid: u32,
    stream: Arc<CudaStream>,
    _module: Arc<CudaModule>,
    relax: CudaFunction,
    fill: CudaFunction,
    seed: CudaFunction,
    cap: usize,
    cap_xy: usize,
    dist: Option<CudaSlice<u32>>,
    stamp: Option<CudaSlice<u32>>,
    q_a: Option<CudaSlice<u32>>,
    q_b: Option<CudaSlice<u32>>,
    cell: Option<CudaSlice<u8>>,
    prox: Option<CudaSlice<u32>>,
    via: Option<CudaSlice<u8>>,
    counter: Option<CudaSlice<u32>>,
    round: u32,
}

static ENGINE: OnceLock<Mutex<Option<Engine>>> = OnceLock::new();
/// pid of the process that loaded the module (set at import). A forked child
/// (the parallel rescue's workers) has another pid and never touches CUDA.
static LOAD_PID: OnceLock<u32> = OnceLock::new();

pub fn note_load_pid() {
    let _ = LOAD_PID.set(std::process::id());
}

/// True when this process may use the GPU flood: it is the process that
/// loaded the module, and a CUDA engine exists or can be created. Checked
/// BEFORE any export work, so a process without a usable GPU pays nothing.
pub fn available() -> bool {
    let me = std::process::id();
    if LOAD_PID.get().copied().unwrap_or(me) != me {
        return false;
    }
    let m = ENGINE.get_or_init(|| Mutex::new(Engine::create()));
    match m.lock() {
        Ok(g) => g.as_ref().map_or(false, |e| e.pid == me),
        Err(_) => false,
    }
}

fn cfg(n: usize) -> LaunchConfig {
    LaunchConfig::for_num_elems(n.max(1) as u32)
}

impl Engine {
    fn create() -> Option<Engine> {
        let ctx = CudaContext::new(0).ok()?;
        let stream = ctx.default_stream();
        let module = ctx.load_module(Ptx::from_src(PTX)).ok()?;
        Some(Engine {
            pid: std::process::id(),
            relax: module.load_function("lee_relax").ok()?,
            fill: module.load_function("lee_fill").ok()?,
            seed: module.load_function("lee_seed").ok()?,
            stream, _module: module,
            cap: 0, cap_xy: 0, dist: None, stamp: None, q_a: None, q_b: None,
            cell: None, prox: None, via: None, counter: None, round: 1,
        })
    }

    fn ensure(&mut self, n: usize, nxy: usize) -> Result<(), String> {
        let st = self.stream.clone();
        let e = |x: cudarc::driver::DriverError| format!("{x:?}");
        if n > self.cap {
            let c = n + n / 4;
            self.dist = Some(st.alloc_zeros::<u32>(c).map_err(e)?);
            self.stamp = Some(st.alloc_zeros::<u32>(c).map_err(e)?);
            self.q_a = Some(st.alloc_zeros::<u32>(c).map_err(e)?);
            self.q_b = Some(st.alloc_zeros::<u32>(c).map_err(e)?);
            self.cell = Some(st.alloc_zeros::<u8>(c).map_err(e)?);
            self.prox = Some(st.alloc_zeros::<u32>(c).map_err(e)?);
            self.cap = c;
            self.round = 1;   // fresh stamp buffer
        }
        if nxy > self.cap_xy {
            let c = nxy + nxy / 4;
            self.via = Some(st.alloc_zeros::<u8>(c).map_err(e)?);
            self.cap_xy = c;
        }
        if self.counter.is_none() {
            self.counter = Some(st.alloc_zeros::<u32>(1).map_err(e)?);
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn flood(&mut self, w: usize, h: usize, l: usize, cell: &[u8], via: &[u8], prox: &[u32],
             lc: &[i32], via_cost: i32, via_prox_mult: i32, seeds: &[u32])
             -> Result<(Vec<u32>, u32), String> {
        let n = w * h * l;
        self.ensure(n, w * h)?;
        let st = self.stream.clone();
        let e = |x: cudarc::driver::DriverError| format!("{x:?}");
        let (wi, hi, li) = (w as i32, h as i32, l as i32);
        let dist = self.dist.as_mut().unwrap();
        let stamp = self.stamp.as_mut().unwrap();
        st.memcpy_htod(cell, &mut self.cell.as_mut().unwrap().slice_mut(0..n)).map_err(e)?;
        st.memcpy_htod(prox, &mut self.prox.as_mut().unwrap().slice_mut(0..n)).map_err(e)?;
        st.memcpy_htod(via, &mut self.via.as_mut().unwrap().slice_mut(0..w * h)).map_err(e)?;
        let d_lc = st.clone_htod(lc).map_err(e)?;
        let nn = n as i64;
        unsafe { st.launch_builder(&self.fill).arg(&mut *dist).arg(&nn).arg(&INF)
            .launch(cfg(n)) }.map_err(e)?;
        // seeds: deduped targets
        let mut sd: Vec<u32> = seeds.to_vec();
        sd.sort_unstable();
        sd.dedup();
        if sd.is_empty() { return Err("no targets in window".into()); }
        self.round = self.round.wrapping_add(1).max(1);
        let d_seed = st.clone_htod(&sd).map_err(e)?;
        let ns = sd.len() as u32;
        unsafe { st.launch_builder(&self.seed).arg(&d_seed).arg(&ns).arg(&mut *dist)
            .arg(&mut *stamp).arg(&self.round).launch(cfg(sd.len())) }.map_err(e)?;
        st.memcpy_htod(&sd, &mut self.q_a.as_mut().unwrap().slice_mut(0..sd.len())).map_err(e)?;
        let mut n_q = ns;
        let mut rounds = 0u32;
        let zero = [0u32];
        while n_q > 0 {
            rounds += 1;
            self.round = self.round.wrapping_add(1).max(1);
            let counter = self.counter.as_mut().unwrap();
            st.memcpy_htod(&zero, &mut *counter).map_err(e)?;
            {
                let (qa, qb) = (self.q_a.as_ref().unwrap(), self.q_b.as_mut().unwrap());
                unsafe { st.launch_builder(&self.relax)
                    .arg(qa).arg(&n_q).arg(qb).arg(&mut *counter)
                    .arg(&mut *dist).arg(&mut *stamp).arg(&self.round)
                    .arg(self.cell.as_ref().unwrap()).arg(self.via.as_ref().unwrap())
                    .arg(self.prox.as_ref().unwrap()).arg(&d_lc)
                    .arg(&wi).arg(&hi).arg(&li).arg(&via_cost).arg(&via_prox_mult)
                    .launch(cfg(n_q as usize)) }.map_err(e)?;
            }
            let c = st.clone_dtoh(&*counter).map_err(e)?;
            n_q = c[0];
            std::mem::swap(&mut self.q_a, &mut self.q_b);
        }
        let out = st.clone_dtoh(&dist.slice(0..n)).map_err(e)?;
        Ok((out, rounds))
    }
}

/// Flood the window; None when no GPU is usable in this process (no device,
/// a forked child, or a CUDA error -- the caller falls back to the CPU).
#[allow(clippy::too_many_arguments)]
pub fn flood(w: usize, h: usize, l: usize, cell: &[u8], via: &[u8], prox: &[u32], lc: &[i32],
             via_cost: i32, via_prox_mult: i32, seeds: &[u32]) -> Option<(Vec<u32>, u32)> {
    let m = ENGINE.get_or_init(|| Mutex::new(Engine::create()));
    let mut g = m.lock().ok()?;
    let eng = g.as_mut()?;
    if eng.pid != std::process::id() {
        return None;
    }
    match eng.flood(w, h, l, cell, via, prox, lc, via_cost, via_prox_mult, seeds) {
        Ok(r) => Some(r),
        Err(msg) => {
            eprintln!("GPU wavefront: {msg}; using the CPU search");
            None
        }
    }
}
