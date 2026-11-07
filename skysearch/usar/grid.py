"""Fast grid path distances (8-connected) using a sparse graph and scipy's C Dijkstra."""
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

_GRAPHS = {}


def _grid_graph(H, W):
    """8-connected grid adjacency (built once per grid size)."""
    if (H, W) not in _GRAPHS:
        idx = np.arange(H * W).reshape(H, W)
        rows, cols = [], []
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == dc == 0:
                    continue
                a = idx[max(0, -dr):H - max(0, dr), max(0, -dc):W - max(0, dc)]
                b = idx[max(0, dr):H + min(0, dr), max(0, dc):W + min(0, dc)]
                rows.append(a.ravel()); cols.append(b.ravel())
        r, c = np.concatenate(rows), np.concatenate(cols)
        _GRAPHS[(H, W)] = csr_matrix((np.ones(len(r)), (r, c)), shape=(H * W, H * W))
    return _GRAPHS[(H, W)]


def dist_field(trav, target):
    """Hop distance (8-connected, in cells) from every cell to target through traversable cells."""
    H, W = trav.shape
    t = trav.copy(); t[target] = True
    keep = np.flatnonzero(t.ravel())
    sub = _grid_graph(H, W)[keep][:, keep]
    ti = int(np.searchsorted(keep, target[0] * W + target[1]))
    D = np.full(H * W, np.inf)
    D[keep] = dijkstra(sub, unweighted=True, indices=ti)
    return D.reshape(H, W)
