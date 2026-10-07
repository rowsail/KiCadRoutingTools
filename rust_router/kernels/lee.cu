// GPU wavefront (Lee / Bellman-Ford worklist) flood for the hybrid search.
//
// Floods BACKWARD from the target cells over a window of the obstacle map and
// leaves, in dist[], the cost of the cheapest FORWARD path from every cell to
// a target under the CPU A*'s move set (8 same-layer directions plus vias at
// one (x, y)) and a LOWER BOUND of its step costs (turn and direction-
// preference terms, both >= 0, are omitted). The CPU search then uses dist[]
// as its heuristic, and a source with no finite dist proves "no path" --
// legality here is PERMISSIVE (a cell is enterable iff !is_blocked, the
// margin-free special case of segment_blocked), so it never forbids a move
// the CPU would allow.
//
// Window cell index  c = (l*H + y)*W + x.
//   cell[c]  bit0: enterable by a track (or a target), bit1: target
//   via[y*W+x] bit0: a via may be placed here, bit1: free via (same-net hole)
//   prox[c]  stub + layer proximity cost of entering c
//   lc[l]    layer cost (1000 = 1.0x); < 0 = forbidden layer

#define INF 0xFFFFFFFFu

__device__ __forceinline__ void relax_to(unsigned* dist, unsigned* stamp, unsigned round,
                                         unsigned* next_q, unsigned* n_next,
                                         unsigned a, unsigned nd) {
    unsigned old = atomicMin(&dist[a], nd);
    if (nd < old) {
        if (atomicExch(&stamp[a], round) != round) {
            unsigned p = atomicAdd(n_next, 1u);
            next_q[p] = a;
        }
    }
}

extern "C" __global__ void lee_relax(
    const unsigned* __restrict__ q, unsigned n_q,
    unsigned* __restrict__ next_q, unsigned* __restrict__ n_next,
    unsigned* __restrict__ dist, unsigned* __restrict__ stamp, unsigned round,
    const unsigned char* __restrict__ cell, const unsigned char* __restrict__ via,
    const unsigned* __restrict__ prox, const int* __restrict__ lc,
    int W, int H, int L, int via_cost, int via_prox_mult)
{
    unsigned i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n_q) return;
    unsigned b = q[i];                       // settled cell: the move's DESTINATION
    unsigned db = dist[b];
    if (db == INF) return;
    if ((cell[b] & 1u) == 0) return;         // nothing can enter b
    int x = b % W;
    unsigned t = b / W;
    int y = t % H;
    int l = t / H;
    unsigned pb = prox[b];

    // Same-layer moves a -> b (a = b - d). The CPU never moves along a
    // forbidden layer.
    if (lc[l] >= 0) {
        long long mult = lc[l];
        const int DX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
        const int DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};
        for (int k = 0; k < 8; k++) {
            int ax = x - DX[k], ay = y - DY[k];
            if (ax < 0 || ay < 0 || ax >= W || ay >= H) continue;
            long long base = (k < 4) ? 1000 : 1414;
            unsigned long long nd = (unsigned long long)db
                + (unsigned long long)(base * mult / 1000) + pb;
            if (nd >= INF) continue;
            relax_to(dist, stamp, round, next_q, n_next,
                     ((unsigned)l * H + ay) * W + ax, (unsigned)nd);
        }
    }
    // Vias a = (x, y, la) -> b = (x, y, l): the via must be placeable (or a
    // free same-net hole) and the destination layer must not be forbidden.
    if (L > 1 && lc[l] >= 0) {
        unsigned char v = via[y * W + x];
        if (v & 3u) {
            bool free_here = (v & 2u) != 0;
            int dl = lc[l];
            for (int la = 0; la < L; la++) {
                if (la == l) continue;
                int sl = lc[la] >= 0 ? lc[la] : 1000;
                int c = (free_here ? 0 : via_cost) + dl - sl;
                if (c < 0) c = 0;
                unsigned long long nd = (unsigned long long)db + (unsigned)c
                    + (unsigned long long)pb * (unsigned long long)via_prox_mult;
                if (nd >= INF) continue;
                relax_to(dist, stamp, round, next_q, n_next,
                         ((unsigned)la * H + y) * W + x, (unsigned)nd);
            }
        }
    }
}

extern "C" __global__ void lee_fill(unsigned* buf, long long n, unsigned v) {
    long long i = (long long)blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) buf[i] = v;
}

// dist = 0 at each seed (target) cell.
extern "C" __global__ void lee_seed(const unsigned* items, unsigned n, unsigned* dist,
                                    unsigned* stamp, unsigned round) {
    unsigned i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    dist[items[i]] = 0;
    stamp[items[i]] = round;
}
