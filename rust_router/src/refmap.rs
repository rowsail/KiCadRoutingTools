//! Dense refcount map for grid cells (replaces `FxHashMap<u64, u16>` for the
//! obstacle map's blocked-cell / blocked-via refcounts).
//!
//! The refcounts are pure bookkeeping -- is_blocked reads the bitmaps -- but
//! every stamp/unstamp/clone went through a hash map: ~40 s of a rescue-heavy
//! route was hash inserts building window maps. Here cells inside a growing
//! window live in a flat `Vec<u16>` (one load/store per cell, clone = memcpy);
//! cells outside a size-capped window fall back to a hash map, so pathological
//! coordinates cost speed, never correctness. A count of 0 means "absent",
//! matching the hash map, which removes an entry when its count drops to 0.

use rustc_hash::FxHashMap;

use crate::types::{pack_xy, unpack_xy};

/// Window growth margin in cells; growth also at least doubles each side.
const GROW_MARGIN: i64 = 64;
/// Cap on window cells (u16 each): 1<<24 cells = 32 MB.
const MAX_CELLS: i64 = 1 << 24;

#[derive(Clone, Default)]
pub struct RefMap {
    min_x: i32,
    min_y: i32,
    width: i32,   // 0 = no window yet
    height: i32,
    counts: Vec<u16>,
    nnz: usize,   // non-zero counts inside the window
    overflow: FxHashMap<u64, u16>,
}

impl RefMap {
    #[inline]
    fn idx(&self, gx: i32, gy: i32) -> Option<usize> {
        let dx = gx.wrapping_sub(self.min_x);
        let dy = gy.wrapping_sub(self.min_y);
        if (dx as u32) < self.width as u32 && (dy as u32) < self.height as u32 {
            // Column-major: a (gx, lo..hi) span is one contiguous run.
            Some(dx as usize * self.height as usize + dy as usize)
        } else {
            None
        }
    }

    /// Grow the window to cover (gx, gy); false if that would exceed the cap
    /// (the caller then uses the overflow map).
    fn grow_to_include(&mut self, gx: i32, gy: i32) -> bool {
        let (gx, gy) = (gx as i64, gy as i64);
        let (mut x0, mut y0, mut x1, mut y1) = if self.width == 0 {
            (gx - GROW_MARGIN, gy - GROW_MARGIN, gx + GROW_MARGIN, gy + GROW_MARGIN)
        } else {
            let (ox0, oy0) = (self.min_x as i64, self.min_y as i64);
            let (ox1, oy1) = (ox0 + self.width as i64 - 1, oy0 + self.height as i64 - 1);
            let (w, h) = (self.width as i64, self.height as i64);
            // Extend only the side(s) that need it, by at least the current
            // extent, so repeated growth is amortised O(1) per cell.
            (if gx < ox0 { (gx - GROW_MARGIN).min(ox0 - w) } else { ox0 },
             if gy < oy0 { (gy - GROW_MARGIN).min(oy0 - h) } else { oy0 },
             if gx > ox1 { (gx + GROW_MARGIN).max(ox1 + w) } else { ox1 },
             if gy > oy1 { (gy + GROW_MARGIN).max(oy1 + h) } else { oy1 })
        };
        // Keep the window inside i32 space.
        x0 = x0.max(i32::MIN as i64); y0 = y0.max(i32::MIN as i64);
        x1 = x1.min(i32::MAX as i64); y1 = y1.min(i32::MAX as i64);
        let (nw, nh) = (x1 - x0 + 1, y1 - y0 + 1);
        if nw * nh > MAX_CELLS {
            // Doubling overshot the cap: retry with just the margin.
            if self.width == 0 {
                return false;
            }
            let (ox0, oy0) = (self.min_x as i64, self.min_y as i64);
            let (ox1, oy1) = (ox0 + self.width as i64 - 1, oy0 + self.height as i64 - 1);
            let (a0, b0) = (ox0.min(gx - GROW_MARGIN), oy0.min(gy - GROW_MARGIN));
            let (a1, b1) = (ox1.max(gx + GROW_MARGIN), oy1.max(gy + GROW_MARGIN));
            if (a1 - a0 + 1) * (b1 - b0 + 1) > MAX_CELLS {
                return false;
            }
            x0 = a0; y0 = b0; x1 = a1; y1 = b1;
        }
        let (nw, nh) = ((x1 - x0 + 1) as i32, (y1 - y0 + 1) as i32);
        let mut counts = vec![0u16; nw as usize * nh as usize];
        if self.width > 0 {
            for dx in 0..self.width {
                let src = dx as usize * self.height as usize;
                let nx = (self.min_x as i64 + dx as i64 - x0) as usize;
                let ny = (self.min_y as i64 - y0) as usize;
                let dst = nx * nh as usize + ny;
                counts[dst..dst + self.height as usize]
                    .copy_from_slice(&self.counts[src..src + self.height as usize]);
            }
        }
        self.min_x = x0 as i32;
        self.min_y = y0 as i32;
        self.width = nw;
        self.height = nh;
        self.counts = counts;
        // Migrate overflow cells that now fall inside the window.
        if !self.overflow.is_empty() {
            let inside: Vec<(u64, u16)> = self.overflow.iter()
                .filter(|(&k, _)| { let (x, y) = unpack_xy(k); self.idx(x, y).is_some() })
                .map(|(&k, &c)| (k, c))
                .collect();
            for (k, c) in inside {
                self.overflow.remove(&k);
                let (x, y) = unpack_xy(k);
                let i = self.idx(x, y).unwrap();
                self.counts[i] = c;
                if c != 0 {
                    self.nnz += 1;
                }
            }
        }
        true
    }

    #[inline]
    fn slot(&mut self, key: u64) -> Option<usize> {
        let (gx, gy) = unpack_xy(key);
        if let Some(i) = self.idx(gx, gy) {
            return Some(i);
        }
        if self.grow_to_include(gx, gy) { self.idx(gx, gy) } else { None }
    }

    /// Add `n` to the cell's count; returns (old, new).
    #[inline]
    pub fn add(&mut self, key: u64, n: u16) -> (u16, u16) {
        match self.slot(key) {
            Some(i) => {
                let old = self.counts[i];
                let new = old.wrapping_add(n);
                self.counts[i] = new;
                if old == 0 && new != 0 { self.nnz += 1; }
                if old != 0 && new == 0 { self.nnz -= 1; }
                (old, new)
            }
            None => {
                let c = self.overflow.entry(key).or_insert(0);
                let old = *c;
                *c = old.wrapping_add(n);
                (old, *c)
            }
        }
    }

    /// Increment every cell of the column span gx, lo..=hi, calling
    /// on_new(gy) for each cell whose count went 0 -> nonzero -- exactly what
    /// inc() per cell would report, but with one window check per span and a
    /// contiguous inner loop.
    #[inline]
    pub fn add_span<F: FnMut(i32)>(&mut self, gx: i32, lo: i32, hi: i32, mut on_new: F) {
        if hi < lo {
            return;
        }
        if self.idx(gx, lo).is_none() { self.grow_to_include(gx, lo); }
        if self.idx(gx, hi).is_none() { self.grow_to_include(gx, hi); }
        let (Some(a), Some(b)) = (self.idx(gx, lo), self.idx(gx, hi)) else {
            // Window capped: per cell (some cells go to the overflow map).
            for gy in lo..=hi {
                let (old, new) = self.add(pack_xy(gx, gy), 1);
                if old == 0 && new != 0 { on_new(gy); }
            }
            return;
        };
        let mut nnz = self.nnz;
        for (k, c) in self.counts[a..=b].iter_mut().enumerate() {
            let old = *c;
            let new = old.wrapping_add(1);
            *c = new;
            if old == 0 {
                nnz += 1;
                on_new(lo + k as i32);
            } else if new == 0 {
                nnz -= 1;
            }
        }
        self.nnz = nnz;
    }

    /// Increment; returns the new count.
    #[inline]
    pub fn inc(&mut self, key: u64) -> u16 {
        self.add(key, 1).1
    }

    /// Decrement a present cell (removing it at 1, as the hash map did).
    /// None = the cell was absent; Some(0) = it was removed.
    #[inline]
    pub fn dec(&mut self, key: u64) -> Option<u16> {
        let (gx, gy) = unpack_xy(key);
        if let Some(i) = self.idx(gx, gy) {
            let c = self.counts[i];
            if c == 0 {
                return None;
            }
            if c > 1 {
                self.counts[i] = c - 1;
                return Some(c - 1);
            }
            self.counts[i] = 0;
            self.nnz -= 1;
            return Some(0);
        }
        match self.overflow.get_mut(&key) {
            None => None,
            Some(c) if *c > 1 => { *c -= 1; Some(*c) }
            Some(_) => { self.overflow.remove(&key); Some(0) }
        }
    }

    #[inline]
    pub fn get(&self, key: &u64) -> Option<u16> {
        let (gx, gy) = unpack_xy(*key);
        match self.idx(gx, gy) {
            Some(i) => { let c = self.counts[i]; if c != 0 { Some(c) } else { None } }
            None => {
                if self.overflow.is_empty() { None } else { self.overflow.get(key).copied() }
            }
        }
    }

    #[inline]
    pub fn contains_key(&self, key: &u64) -> bool {
        self.get(key).is_some()
    }

    pub fn insert(&mut self, key: u64, v: u16) {
        let old = self.get(&key).unwrap_or(0);
        if v >= old {
            self.add(key, v - old);
        } else {
            self.add(key, (v as u32 + 0x10000 - old as u32) as u16);
        }
    }

    pub fn len(&self) -> usize {
        self.nnz + self.overflow.len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn clear(&mut self) {
        *self = RefMap::default();
    }

    pub fn shrink_to_fit(&mut self) {
        self.overflow.shrink_to_fit();
    }

    /// (key, count) for every present cell; order is unspecified (as the hash
    /// map's was).
    pub fn iter(&self) -> impl Iterator<Item = (u64, u16)> + '_ {
        let h = self.height.max(1) as usize;
        let (x0, y0) = (self.min_x, self.min_y);
        self.counts.iter().enumerate()
            .filter(|(_, &c)| c != 0)
            .map(move |(i, &c)| (pack_xy(x0 + (i / h) as i32, y0 + (i % h) as i32), c))
            .chain(self.overflow.iter().map(|(&k, &c)| (k, c)))
    }

    pub fn keys(&self) -> impl Iterator<Item = u64> + '_ {
        self.iter().map(|(k, _)| k)
    }

    pub fn values(&self) -> impl Iterator<Item = u16> + '_ {
        self.iter().map(|(_, c)| c)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn add_span_matches_inc() {
        let mut seed: u64 = 0x9E3779B97F4A7C15;
        let mut rnd = || { seed ^= seed << 13; seed ^= seed >> 7; seed ^= seed << 17; seed };
        let mut a = RefMap::default();
        let mut b = RefMap::default();
        for step in 0..60_000u32 {
            let r = rnd();
            let gx = if r % 211 == 0 { ((r >> 9) as i32) % 1_000_000_000 }
                     else { (r % 2000) as i32 - 1000 + (step as i32 / 500) };
            let lo = ((r >> 20) % 1500) as i32 - 750;
            let hi = lo + ((r >> 40) % 60) as i32 - 3;          // includes empty spans
            let mut ta = Vec::new();
            a.add_span(gx, lo, hi, |gy| ta.push(gy));
            let mut tb = Vec::new();
            if hi >= lo { for gy in lo..=hi { if b.inc(pack_xy(gx, gy)) == 1 { tb.push(gy); } } }
            assert_eq!(ta, tb);
            if r % 7 == 0 { let k = pack_xy(gx, lo); assert_eq!(a.dec(k), b.dec(k)); }
        }
        assert_eq!(a.len(), b.len());
        let mut x: Vec<(u64, u16)> = a.iter().collect(); x.sort();
        let mut y: Vec<(u64, u16)> = b.iter().collect(); y.sort();
        assert_eq!(x, y);
    }

    #[test]
    fn matches_hashmap() {
        // Differential test against the FxHashMap semantics it replaces.
        let mut seed: u64 = 0x2545F4914F6CDD1D;
        let mut rnd = || { seed ^= seed << 13; seed ^= seed >> 7; seed ^= seed << 17; seed };
        let mut a = RefMap::default();
        let mut b: FxHashMap<u64, u16> = FxHashMap::default();
        for step in 0..400_000u32 {
            let r = rnd();
            // mostly clustered cells, sometimes far away (forces overflow)
            let (gx, gy) = if r % 97 == 0 {
                (((r >> 8) as i32) % 2_000_000_000, ((r >> 40) as i32) % 2_000_000_000)
            } else {
                ((r % 3000) as i32 - 1500 + (step as i32 / 1000), ((r >> 16) % 3000) as i32 - 1500)
            };
            let key = pack_xy(gx, gy);
            match (r >> 60) % 4 {
                0 | 1 => {
                    let n = a.inc(key);
                    let e = b.entry(key).or_insert(0); *e += 1;
                    assert_eq!(n, *e);
                }
                2 => {
                    let got = a.dec(key);
                    let want = match b.get_mut(&key) {
                        None => None,
                        Some(c) if *c > 1 => { *c -= 1; Some(*c) }
                        Some(_) => { b.remove(&key); Some(0) }
                    };
                    assert_eq!(got, want);
                }
                _ => {
                    assert_eq!(a.get(&key), b.get(&key).copied());
                }
            }
            if step % 50_000 == 0 {
                assert_eq!(a.len(), b.len());
                let mut x: Vec<(u64, u16)> = a.iter().collect(); x.sort();
                let mut y: Vec<(u64, u16)> = b.iter().map(|(&k, &c)| (k, c)).collect(); y.sort();
                assert_eq!(x, y);
                let c = a.clone();
                let mut z: Vec<(u64, u16)> = c.iter().collect(); z.sort();
                assert_eq!(z, y);
            }
        }
    }
}
