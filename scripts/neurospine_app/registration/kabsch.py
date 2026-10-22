import math
import numpy as np


def kabsch(P_src, P_dst):
    P_src = np.asarray(P_src, dtype=np.float64)
    P_dst = np.asarray(P_dst, dtype=np.float64)

    if P_src.shape != P_dst.shape:
        raise ValueError(f"Formes incompatibles: {P_src.shape} vs {P_dst.shape}")
    if P_src.shape[1] != 3:
        raise ValueError(f"Els punts han de ser (N, 3), no {P_src.shape}")
    if P_src.shape[0] < 3:
        raise ValueError(f"Calen ≥ 3 punts, se n'han donat {P_src.shape[0]}")

    c_src = P_src.mean(axis=0)
    c_dst = P_dst.mean(axis=0)

    Q_src = P_src - c_src
    Q_dst = P_dst - c_dst

    H = Q_src.T @ Q_dst
    U, S, Vt = np.linalg.svd(H)

    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T

    t = c_dst - R @ c_src

    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = t
    return T


def apply_T(T, points):
    T = np.asarray(T, dtype=np.float64)
    points = np.asarray(points, dtype=np.float64)

    single = (points.ndim == 1)
    if single:
        points = points[None, :]

    N = points.shape[0]
    homog = np.hstack([points, np.ones((N, 1))])
    transformed = (T @ homog.T).T[:, :3]
    return transformed[0] if single else transformed


def residuals(T, P_src, P_dst):
    P_est = apply_T(T, P_src)
    return np.linalg.norm(P_est - np.asarray(P_dst), axis=1)


def fit_quality(T, P_src, P_dst):
    errs = residuals(T, P_src, P_dst)
    return {
        "rmse":      float(np.sqrt((errs ** 2).mean())),
        "mean":      float(errs.mean()),
        "max":       float(errs.max()),
        "per_point": errs,
    }


def inverse_T(T):
    T = np.asarray(T, dtype=np.float64)
    T_inv = np.eye(4)
    R = T[:3, :3]
    t = T[:3,  3]
    T_inv[:3, :3] = R.T
    T_inv[:3,  3] = -R.T @ t
    return T_inv


class SurfaceLocator:
    def __init__(self, polydata):
        import vtk
        self._vtk = vtk
        self._polydata = polydata

        self._locator = vtk.vtkCellLocator()
        self._locator.SetDataSet(polydata)
        self._locator.BuildLocator()

        self._closest = [0.0, 0.0, 0.0]
        self._cellId  = vtk.reference(0)
        self._subId   = vtk.reference(0)
        self._dist2   = vtk.reference(0.0)

    def closest_point(self, pt):
        p = [float(pt[0]), float(pt[1]), float(pt[2])]
        self._locator.FindClosestPoint(
            p, self._closest, self._cellId, self._subId, self._dist2)
        return np.array(self._closest, dtype=np.float64)

    def distance_to(self, pt):
        p = [float(pt[0]), float(pt[1]), float(pt[2])]
        self._locator.FindClosestPoint(
            p, self._closest, self._cellId, self._subId, self._dist2)
        return math.sqrt(self._dist2.get())

    def closest_points_batch(self, pts):
        out = np.zeros_like(pts, dtype=np.float64)
        for i, p in enumerate(pts):
            out[i] = self.closest_point(p)
        return out


def icp_refine(P_captured, surface_locator, T_init,
                max_iterations=50, tolerance_mm=0.01,
                reject_ratio=None):
    P_captured = np.asarray(P_captured, dtype=np.float64)
    T = np.asarray(T_init, dtype=np.float64).copy()

    history = []
    prev_mean_shift = None

    for it in range(max_iterations):
        T_inv = inverse_T(T)
        P_in_ct = apply_T(T_inv, P_captured)

        Q_correspondents = surface_locator.closest_points_batch(P_in_ct)

        dists = np.linalg.norm(P_in_ct - Q_correspondents, axis=1)

        if reject_ratio is not None and 0 < reject_ratio < 1:
            keep_n = max(3, int(len(dists) * (1.0 - reject_ratio)))
            keep_idx = np.argsort(dists)[:keep_n]
            src_used = Q_correspondents[keep_idx]
            dst_used = P_captured[keep_idx]
        else:
            src_used = Q_correspondents
            dst_used = P_captured

        if len(src_used) < 3:
            break
        T_new = kabsch(src_used, dst_used)

        P_new = apply_T(T_new, Q_correspondents)
        P_old = apply_T(T,     Q_correspondents)
        mean_shift = float(np.linalg.norm(P_new - P_old, axis=1).mean())

        history.append({
            "iter": it,
            "mean_dist_to_surface_mm": float(dists.mean()),
            "rms_dist_to_surface_mm": float(np.sqrt((dists ** 2).mean())),
            "mean_shift_mm": mean_shift,
        })

        T = T_new

        if mean_shift < tolerance_mm:
            break
        if prev_mean_shift is not None and abs(prev_mean_shift - mean_shift) < 1e-6:
            break
        prev_mean_shift = mean_shift

    return T, history


def surface_fit_quality(P_captured, surface_locator, T):
    P_captured = np.asarray(P_captured, dtype=np.float64)
    T_inv = inverse_T(T)
    P_in_ct = apply_T(T_inv, P_captured)
    dists = np.array([surface_locator.distance_to(p) for p in P_in_ct])
    return {
        "rmse":      float(np.sqrt((dists ** 2).mean())),
        "mean":      float(dists.mean()),
        "max":       float(dists.max()),
        "per_point": dists,
    }

