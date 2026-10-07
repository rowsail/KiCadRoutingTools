// GPU negotiated-congestion (PathFinder) global router kernels.
//
// Board grid: W x H cells x L layers, board cell index  c = (l*H + y)*W + x.
// Each net being searched owns a "slot": a window (x0, y0, ww, wh) over all
// layers, with window-local index  li = (l*wh + (y-y0))*ww + (x-x0), and its
// own slices of the dist / stamp / bit buffers at per-slot offsets.
//
// dist[] holds packed u64 (cost << 32 | predecessor local index); atomicMin
// on the packed value keeps the cheapest cost and, on ties, the smallest
// predecessor -- deterministic regardless of thread order.

#define INF64 0xFFFFFFFFFFFFFFFFull
#define NO_PRED 0xFFFFFFFFu

struct Slot {
    int x0, y0, ww, wh;      // window
    long long off;           // offset into dist / stamp (cells)
    long long boff;          // offset into bit buffers (u32 words)
};

__device__ __forceinline__ bool bit_get(const unsigned* b, long long boff, unsigned li) {
    return (b[boff + (li >> 5)] >> (li & 31)) & 1u;
}

// Relax one NEAR-queue item (slot, local idx): 8 same-layer neighbours plus
// layer changes through a via at the same (x, y). Near-far (delta-stepping):
// an improved neighbour goes to the near queue when its cost is below the
// current threshold T, else to the far queue. Items of a slot that already
// reached a target cheaper than this item are dropped (its search is done).
//
// blocked[]: 0 free, 1 soft (other nets' keep-outs: passable at block_pen x
// cost, so boxed-in fine-pitch pins can always escape), 2 hard (board edge,
// outline, cutouts). Target cells are always enterable.
__device__ __forceinline__ void push_item(unsigned long long cand, unsigned s, unsigned nli,
        unsigned long long* dist, long long o, unsigned* stamp, unsigned round, unsigned T,
        unsigned long long* near_q, unsigned long long* far_q, unsigned* counters,
        unsigned far_cap, bool is_tgt, unsigned* best_tgt) {
    unsigned long long old = atomicMin(&dist[o], cand);
    if (cand >= old) return;
    unsigned c = (unsigned)(cand >> 32);
    if (is_tgt) atomicMin(&best_tgt[s], c);
    unsigned long long item = ((unsigned long long)s << 32) | nli;
    if (c < T) {
        if (atomicExch(&stamp[o], round) != round) {
            unsigned p = atomicAdd(&counters[0], 1u);
            near_q[p] = item;
        }
    } else {
        unsigned p = atomicAdd(&counters[1], 1u);
        if (p < far_cap) far_q[p] = item;
    }
}

extern "C" __global__ void relax(
    const unsigned long long* __restrict__ frontier, unsigned n_front,
    unsigned long long* __restrict__ near_q, unsigned long long* __restrict__ far_q,
    unsigned* __restrict__ counters, unsigned far_cap,   // counters: [n_near, n_far]
    unsigned* __restrict__ stamp, unsigned round, unsigned T,
    unsigned long long* __restrict__ dist,
    const Slot* __restrict__ slots,
    const unsigned char* __restrict__ blocked,      // [L][H][W] 0/1/2
    const unsigned char* __restrict__ via_blocked,  // [H][W] 0/1/2
    const float* __restrict__ hist,                 // [L][H][W]
    const unsigned* __restrict__ occ,               // [L][H][W]
    const unsigned* __restrict__ tgt_bits,          // per slot window bits
    unsigned* __restrict__ best_tgt,                // per slot
    const int* __restrict__ layer_cost,             // per layer, 1000 = 1.0
    int W, int H, int L, int via_cost, float pres, float block_pen)
{
    unsigned i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n_front) return;
    unsigned long long item = frontier[i];
    unsigned s = (unsigned)(item >> 32);
    unsigned li = (unsigned)(item & 0xFFFFFFFFu);
    Slot S = slots[s];
    unsigned long long cur = dist[S.off + li];
    unsigned cost = (unsigned)(cur >> 32);
    if (cost >= best_tgt[s]) return;               // this net's search is done
    int lx = li % S.ww;
    unsigned t = li / S.ww;
    int ly = t % S.wh;
    int l = t / S.wh;
    int gx = lx + S.x0, gy = ly + S.y0;

    const int DX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    for (int k = 0; k < 8; k++) {
        int nx = gx + DX[k], ny = gy + DY[k];
        int nlx = nx - S.x0, nly = ny - S.y0;
        if (nlx < 0 || nly < 0 || nlx >= S.ww || nly >= S.wh) continue;
        if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
        long long bc = ((long long)l * H + ny) * W + nx;
        unsigned nli = ((unsigned)l * S.wh + nly) * S.ww + nlx;
        bool is_tgt = bit_get(tgt_bits, S.boff, nli);
        unsigned char bl = blocked[bc];
        if (bl == 2 && !is_tgt) continue;
        float step = (k < 4 ? 1000.0f : 1414.0f) * layer_cost[l] * 0.001f;
        if (bl == 1 && !is_tgt) step *= block_pen;
        int others = (int)occ[bc];
        float mult = (1.0f + hist[bc]) * (1.0f + pres * (others > 0 ? others : 0));
        float add = step * mult;
        if (add > 2.0e8f) add = 2.0e8f;
        unsigned long long nc = (unsigned long long)cost + (unsigned long long)add;
        if (nc > 0xFFFFFFF0ull) continue;
        push_item((nc << 32) | li, s, nli, dist, S.off + nli, stamp, round, T,
                  near_q, far_q, counters, far_cap, is_tgt, best_tgt);
    }
    unsigned char vbl = via_blocked[(long long)gy * W + gx];
    if (L > 1 && vbl != 2) {
        float vpen = (vbl == 1) ? block_pen : 1.0f;
        for (int l2 = 0; l2 < L; l2++) {
            if (l2 == l) continue;
            long long bc = ((long long)l2 * H + gy) * W + gx;
            unsigned nli = ((unsigned)l2 * S.wh + ly) * S.ww + lx;
            bool is_tgt = bit_get(tgt_bits, S.boff, nli);
            unsigned char bl = blocked[bc];
            if (bl == 2 && !is_tgt) continue;
            int others = (int)occ[bc];
            float mult = (1.0f + hist[bc]) * (1.0f + pres * (others > 0 ? others : 0));
            float add = (float)via_cost * vpen * ((bl == 1 && !is_tgt) ? block_pen : 1.0f) * mult;
            if (add > 2.0e8f) add = 2.0e8f;
            unsigned long long nc = (unsigned long long)cost + (unsigned long long)add;
            if (nc > 0xFFFFFFF0ull) continue;
            push_item((nc << 32) | li, s, nli, dist, S.off + nli, stamp, round, T,
                      near_q, far_q, counters, far_cap, is_tgt, best_tgt);
        }
    }
}

// Advance the threshold: move far items whose CURRENT cost is below T (and
// whose slot is not done) into the near queue (deduped by stamp); keep the
// rest (still current, cost >= T) in the far queue. Stale duplicates of a
// cell are dropped by the stamp dedup.
// counters: [0] n_near, [2] n_far_out, [3] min far cost.
extern "C" __global__ void split_far(const unsigned long long* far_in, unsigned n_in,
                                     unsigned long long* near_q, unsigned long long* far_out,
                                     unsigned* counters,
                                     unsigned* stamp, unsigned round, unsigned T,
                                     const unsigned long long* dist, const Slot* slots,
                                     const unsigned* best_tgt) {
    unsigned i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n_in) return;
    unsigned long long item = far_in[i];
    unsigned s = (unsigned)(item >> 32);
    unsigned li = (unsigned)(item & 0xFFFFFFFFu);
    long long o = slots[s].off + li;
    unsigned c = (unsigned)(dist[o] >> 32);
    if (c >= best_tgt[s]) return;                  // slot done
    if (c < T) {
        if (atomicExch(&stamp[o], round) != round) {
            unsigned p = atomicAdd(&counters[0], 1u);
            near_q[p] = item;
        }
    } else {
        unsigned p = atomicAdd(&counters[2], 1u);
        far_out[p] = item;
        atomicMin(&counters[3], c);
    }
}

// Fill a range of a u64 buffer with a value.
extern "C" __global__ void fill_u64(unsigned long long* buf, long long n, unsigned long long v) {
    long long i = (long long)blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) buf[i] = v;
}

extern "C" __global__ void fill_u32(unsigned* buf, long long n, unsigned v) {
    long long i = (long long)blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) buf[i] = v;
}

// Seed sources: items are (slot << 32 | local idx); dist = 0, no predecessor.
extern "C" __global__ void seed(const unsigned long long* items, unsigned n,
                                unsigned long long* dist, const Slot* slots,
                                unsigned* stamp, unsigned round) {
    unsigned i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    unsigned s = (unsigned)(items[i] >> 32);
    unsigned li = (unsigned)(items[i] & 0xFFFFFFFFu);
    long long o = slots[s].off + li;
    dist[o] = (unsigned long long)NO_PRED;          // cost 0, no predecessor
    stamp[o] = round;
}

// Set bits for (slot, local idx) items.
extern "C" __global__ void set_bits(const unsigned long long* items, unsigned n,
                                    unsigned* bits, const Slot* slots) {
    unsigned i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    unsigned s = (unsigned)(items[i] >> 32);
    unsigned li = (unsigned)(items[i] & 0xFFFFFFFFu);
    atomicOr(&bits[slots[s].boff + (li >> 5)], 1u << (li & 31));
}

// Read dist for (slot, local idx) items (target selection on the host).
extern "C" __global__ void gather_dist(const unsigned long long* items, unsigned n,
                                       const unsigned long long* dist, const Slot* slots,
                                       unsigned long long* out) {
    unsigned i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    unsigned s = (unsigned)(items[i] >> 32);
    unsigned li = (unsigned)(items[i] & 0xFFFFFFFFu);
    out[i] = dist[slots[s].off + li];
}

// Walk predecessors from each (slot, local idx) start; one thread per path.
// Writes local indices into out[k*max_len ...] and the length into lens[k].
extern "C" __global__ void backtrack(const unsigned long long* starts, unsigned n,
                                     const unsigned long long* dist, const Slot* slots,
                                     unsigned* out, unsigned* lens, unsigned max_len) {
    unsigned k = blockIdx.x * blockDim.x + threadIdx.x;
    if (k >= n) return;
    unsigned s = (unsigned)(starts[k] >> 32);
    unsigned li = (unsigned)(starts[k] & 0xFFFFFFFFu);
    long long off = slots[s].off;
    unsigned len = 0;
    while (len < max_len) {
        out[(long long)k * max_len + len] = li;
        len++;
        unsigned pred = (unsigned)(dist[off + li] & 0xFFFFFFFFull);
        if (pred == NO_PRED) break;
        li = pred;
    }
    lens[k] = len;
}

// Stamp route occupancy. Each item is (slot, local idx of a route cell,
// radius r, via flag). Every board cell within r (same layer; all layers for
// a via) is marked in the slot's "cur" own-bits; the first time a slot marks
// a cell, occ[cell] += 1 -- so occ counts DISTINCT nets per cell.
extern "C" __global__ void stamp_occ(const unsigned long long* items, const int* radius,
                                     const unsigned char* is_via, unsigned n,
                                     unsigned* own_cur, unsigned* occ, const Slot* slots,
                                     const unsigned char* pin_mask,
                                     int W, int H, int L) {
    unsigned i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    unsigned s = (unsigned)(items[i] >> 32);
    unsigned li = (unsigned)(items[i] & 0xFFFFFFFFu);
    Slot S = slots[s];
    int lx = li % S.ww; unsigned t = li / S.ww; int ly = t % S.wh; int l = t / S.wh;
    int r = radius[i];
    int l_lo = is_via[i] ? 0 : l, l_hi = is_via[i] ? L - 1 : l;
    for (int ll = l_lo; ll <= l_hi; ll++)
    for (int dy = -r; dy <= r; dy++)
    for (int dx = -r; dx <= r; dx++) {
        if (dx * dx + dy * dy > r * r) continue;
        int x = lx + dx, y = ly + dy;
        if (x < 0 || y < 0 || x >= S.ww || y >= S.wh) continue;
        int gx = x + S.x0, gy = y + S.y0;
        if (gx < 0 || gy < 0 || gx >= W || gy >= H) continue;
        // Pad areas never count: neighbouring fine-pitch pins of different
        // nets overlap there by construction (the detailed router owns it).
        if (pin_mask[((long long)ll * H + gy) * W + gx]) continue;
        unsigned nli = ((unsigned)ll * S.wh + y) * S.ww + x;
        unsigned m = 1u << (nli & 31);
        unsigned old = atomicOr(&own_cur[S.boff + (nli >> 5)], m);
        if (!(old & m)) atomicAdd(&occ[((long long)ll * H + gy) * W + gx], 1u);
    }
}

// hist += hfac * overuse; count overused cells.
extern "C" __global__ void update_hist(const unsigned* occ, float* hist, long long n,
                                       float hfac, unsigned* n_over) {
    long long i = (long long)blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    unsigned o = occ[i];
    if (o > 1) {
        hist[i] += hfac * (float)(o - 1);
        atomicAdd(n_over, 1u);
    }
}

// Remove the listed slots' current occupancy: for every set own-bit, occ -= 1,
// then clear the bits. One block per listed slot.
extern "C" __global__ void unstamp(const unsigned* slot_list, unsigned* own, unsigned* occ,
                                   const Slot* slots, int W, int H, int L) {
    unsigned s = slot_list[blockIdx.x];
    Slot S = slots[s];
    long long cells = (long long)S.ww * S.wh * L;
    long long nw = (cells + 31) / 32;
    for (long long wi = threadIdx.x; wi < nw; wi += blockDim.x) {
        unsigned word = own[S.boff + wi];
        if (!word) continue;
        own[S.boff + wi] = 0;
        while (word) {
            int b = __ffs(word) - 1;
            word &= word - 1;
            unsigned li = (unsigned)(wi * 32 + b);
            int lx = li % S.ww; unsigned t = li / S.ww; int ly = t % S.wh; int l = t / S.wh;
            atomicSub(&occ[((long long)l * H + (ly + S.y0)) * W + (lx + S.x0)], 1u);
        }
    }
}

// flag[s] = 1 when any cell of slot s's current route halo is overused.
// One block per slot.
extern "C" __global__ void flag_congested(const unsigned* own, const unsigned* occ,
                                          const Slot* slots, unsigned* flag,
                                          int W, int H, int L) {
    unsigned s = blockIdx.x;
    Slot S = slots[s];
    long long cells = (long long)S.ww * S.wh * L;
    long long nw = (cells + 31) / 32;
    for (long long wi = threadIdx.x; wi < nw; wi += blockDim.x) {
        unsigned word = own[S.boff + wi];
        while (word) {
            int b = __ffs(word) - 1;
            word &= word - 1;
            unsigned li = (unsigned)(wi * 32 + b);
            int lx = li % S.ww; unsigned t = li / S.ww; int ly = t % S.wh; int l = t / S.wh;
            if (occ[((long long)l * H + (ly + S.y0)) * W + (lx + S.x0)] > 1) { flag[s] = 1; return; }
        }
    }
}

// Count overused cells and add history: hist += hfac * (occ - 1).
extern "C" __global__ void count_over(const unsigned* occ, long long n, unsigned* n_over) {
    long long i = (long long)blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n && occ[i] > 1) atomicAdd(n_over, 1u);
}

// Set the 4 queue counters without a host copy (async; no sync point).
extern "C" __global__ void set_counters(unsigned* c, unsigned a, unsigned b, unsigned d, unsigned e) {
    c[0] = a; c[1] = b; c[2] = d; c[3] = e;
}
