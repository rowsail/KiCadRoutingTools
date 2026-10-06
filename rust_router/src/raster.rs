//! Grid rasterizers ported from py_router/routing_utils.py.
//!
//! Each is an operation-for-operation port of the numpy original (same f64
//! expressions in the same order, no fused multiply-add), so the cells it
//! returns are bit-identical; routing_utils keeps the numpy code as the
//! fallback and the definition of record.

use numpy::ndarray::Array2;
use numpy::{IntoPyArray, PyArray2};
use pyo3::exceptions::PyAssertionError;
use pyo3::prelude::*;

/// routing_utils.GRID_TIE_EPS
const GRID_TIE_EPS: f64 = 1e-6;

/// Port of routing_utils.segment_blocked_spans' miss path (_capsule_mask +
/// span extraction): (M, 3) int32 rows (gx, gy_lo, gy_hi), both inclusive,
/// for every column with a cell whose centre is strictly within
/// `margin - GRID_TIE_EPS` of the segment (x1,y1)-(x2,y2).
#[pyfunction]
pub fn capsule_spans<'py>(py: Python<'py>, x1: f64, y1: f64, x2: f64, y2: f64,
                          margin: f64, grid_step: f64)
                          -> PyResult<Bound<'py, PyArray2<i32>>> {
    let rows = capsule_spans_rows(x1, y1, x2, y2, margin, grid_step)
        .ok_or_else(|| PyAssertionError::new_err("capsule column not contiguous"))?;
    let n = rows.len() / 3;
    Ok(Array2::from_shape_vec((n, 3), rows).expect("shape").into_pyarray(py))
}

pub fn capsule_spans_rows(x1: f64, y1: f64, x2: f64, y2: f64,
                          margin: f64, grid_step: f64) -> Option<Vec<i32>> {
    let inv = 1.0 / grid_step;
    let glo_x = ((x1.min(x2) - margin) * inv).floor() as i64;
    let ghi_x = ((x1.max(x2) + margin) * inv).ceil() as i64;
    let glo_y = ((y1.min(y2) - margin) * inv).floor() as i64;
    let ghi_y = ((y1.max(y2) + margin) * inv).ceil() as i64;
    let dx = x2 - x1;
    let dy = y2 - y1;
    let l2 = dx * dx + dy * dy;
    let m_eff = margin - GRID_TIE_EPS;
    let m2 = m_eff * m_eff;
    let mut out = Vec::new();
    for gx in glo_x..=ghi_x {
        // numpy: arange(int32) -> astype(float64) * grid_step
        let cx = (gx as i32) as f64 * grid_step;
        let (mut lo, mut hi, mut count) = (i64::MAX, i64::MIN, 0i64);
        for gy in glo_y..=ghi_y {
            let cy = (gy as i32) as f64 * grid_step;
            let t = if l2 <= 0.0 {
                0.0
            } else {
                let t = ((cx - x1) * dx + (cy - y1) * dy) / l2;
                // np.clip(t, 0.0, 1.0)
                if t < 0.0 { 0.0 } else if t > 1.0 { 1.0 } else { t }
            };
            let ddx = cx - (x1 + t * dx);
            let ddy = cy - (y1 + t * dy);
            if ddx * ddx + ddy * ddy < m2 {
                if gy < lo { lo = gy; }
                hi = gy;
                count += 1;
            }
        }
        if count > 0 {
            if hi - lo + 1 != count {
                return None;   // the numpy original raises AssertionError
            }
            out.push(gx as i32);
            out.push(lo as i32);
            out.push(hi as i32);
        }
    }
    Some(out)
}

/// Port of routing_utils.pad_blocked_cells_array's miss path: the (N, 2)
/// int32 cell OFFSETS (relative to the pad's grid cell) the pad keep-out
/// blocks, in the numpy original's order (ex outer, ey inner).
#[pyfunction]
#[allow(clippy::too_many_arguments)]
pub fn pad_offsets<'py>(py: Python<'py>, half_width: f64, half_height: f64,
                        margin: f64, grid_step: f64, corner_radius: f64,
                        corner_buffer: f64, off_x: f64, off_y: f64,
                        rotation_deg: f64) -> Bound<'py, PyArray2<i32>> {
    let rows = pad_offsets_rows(half_width, half_height, margin, grid_step,
                                corner_radius, corner_buffer, off_x, off_y,
                                rotation_deg);
    let n = rows.len() / 2;
    Array2::from_shape_vec((n, 2), rows).expect("shape").into_pyarray(py)
}

#[allow(clippy::too_many_arguments)]
pub fn pad_offsets_rows(half_width: f64, half_height: f64, margin: f64,
                        grid_step: f64, corner_radius: f64, corner_buffer: f64,
                        off_x: f64, off_y: f64, rotation_deg: f64) -> Vec<i32> {
    let rotated = rotation_deg.abs() > 1e-9 && (rotation_deg.abs() - 180.0).abs() > 1e-9;
    let (mut rc, mut rs) = (1.0f64, 0.0f64);
    let (gext_x, gext_y) = if rotated {
        // math.radians: x * (pi / 180)
        let rrad = rotation_deg * (std::f64::consts::PI / 180.0);
        rc = rrad.cos();
        rs = rrad.sin();
        ((half_width * rc).abs() + (half_height * rs).abs(),
         (half_width * rs).abs() + (half_height * rc).abs())
    } else {
        (half_width, half_height)
    };
    let expand_x = ((gext_x + margin + corner_buffer) / grid_step).ceil() as i64 + 1;
    let expand_y = ((gext_y + margin + corner_buffer) / grid_step).ceil() as i64 + 1;
    let margin_sq = margin * margin;
    let buffered_margin_sq = (margin + corner_buffer) * (margin + corner_buffer);
    let corner_threshold = if corner_radius == 0.0 { grid_step / 2.0 } else { 0.0 };
    let inner_half_w = half_width - corner_radius - corner_threshold;
    let inner_half_h = half_height - corner_radius - corner_threshold;
    let cr_inner_w = half_width - corner_radius;
    let cr_inner_h = half_height - corner_radius;
    let mut out = Vec::new();
    for ex in -expand_x..=expand_x {
        let ex_f = ex as f64 * grid_step - off_x;
        for ey in -expand_y..=expand_y {
            let mut cell_x = ex_f;
            let mut cell_y = ey as f64 * grid_step - off_y;
            if rotated {
                let lx = cell_x * rc + cell_y * rs;
                let ly = -cell_x * rs + cell_y * rc;
                cell_x = lx;
                cell_y = ly;
            }
            let abs_x = cell_x.abs();
            let abs_y = cell_y.abs();
            let in_corner = abs_x > inner_half_w && abs_y > inner_half_h;
            let eff = if in_corner { buffered_margin_sq } else { margin_sq };
            let (corner_region, corner_dist_sq) = if corner_radius > 0.0 {
                let region = abs_x > cr_inner_w && abs_y > cr_inner_h;
                let dxc = abs_x - cr_inner_w;
                let dyc = abs_y - cr_inner_h;
                let dist = (dxc * dxc + dyc * dyc).sqrt() - corner_radius;
                (region, if dist > 0.0 { dist * dist } else { 0.0 })
            } else {
                (false, 0.0)
            };
            // np.maximum(-hw, np.minimum(cell, hw))
            let closest_x = (-half_width).max(cell_x.min(half_width));
            let closest_y = (-half_height).max(cell_y.min(half_height));
            let dx = cell_x - closest_x;
            let dy = cell_y - closest_y;
            let rect_dist_sq = dx * dx + dy * dy;
            let dist_sq = if corner_region { corner_dist_sq } else { rect_dist_sq };
            if dist_sq < eff {
                out.push(ex as i32);
                out.push(ey as i32);
            }
        }
    }
    out
}

extern "C" {
    fn pow(x: f64, y: f64) -> f64;
}

/// CPython's `float ** float` for the cases _disk_spans uses (exponent 2.0
/// or 0.5, finite base): CPython takes |x| for a negative base with an
/// integer exponent, short-cuts 1.0, then calls the C library pow. Calling
/// that same pow -- with the exponent opaque, so LLVM cannot rewrite
/// pow(x, 2.0) as x*x -- keeps the result bit-identical to the Python code.
#[inline]
fn py_pow(x: f64, y: f64) -> f64 {
    let x = if x < 0.0 { -x } else { x };
    if x == 1.0 {
        return 1.0;
    }
    unsafe { pow(x, std::hint::black_box(y)) }
}

/// Port of bga_fanout.underpad._Occ._disk_spans: column spans
/// [(a, b_lo, b_hi)] of the grid cells within r of the real point (x, y),
/// clipped to the nx x ny grid with origin (x0, y0) and cell size res.
#[pyfunction]
#[allow(clippy::too_many_arguments)]
pub fn disk_spans(x: f64, y: f64, r: f64, x0: f64, y0: f64, res: f64,
                  nx: i64, ny: i64) -> Vec<(i64, i64, i64)> {
    let fx = (x - x0) / res;
    let fy = (y - y0) / res;
    let (cx, cy) = (fx as i64, fy as i64);           // int(): truncation
    let rc = (r / res) as i64 + 2;
    let thr = py_pow(r / res, 2.0);
    let a0 = (cx - rc).max(0);
    let a1 = (cx + rc).min(nx - 1);
    let blo_lim = (cy - rc).max(0);
    let bhi_lim = (cy + rc).min(ny - 1);
    let pred = |a: i64, b: i64| py_pow(a as f64 - fx, 2.0) + py_pow(b as f64 - fy, 2.0);
    let mut spans = Vec::new();
    for a in a0..=a1 {
        let adx2 = py_pow(a as f64 - fx, 2.0);
        let rem = thr - adx2;
        if rem < 0.0 {
            continue;
        }
        let half = py_pow(rem, 0.5);
        let mut b0 = (fy - half) as i64 - 1;
        let mut b1 = (fy + half) as i64 + 1;
        if b0 < blo_lim { b0 = blo_lim; }
        if b1 > bhi_lim { b1 = bhi_lim; }
        while b0 <= b1 && pred(a, b0) > thr { b0 += 1; }
        while b1 >= b0 && pred(a, b1) > thr { b1 -= 1; }
        if b0 > b1 {
            continue;
        }
        while b0 - 1 >= blo_lim && pred(a, b0 - 1) <= thr { b0 -= 1; }
        while b1 + 1 <= bhi_lim && pred(a, b1 + 1) <= thr { b1 += 1; }
        spans.push((a, b0, b1));
    }
    spans
}
