"""
Registre rígid entre dos conjunts de punts (Kabsch/SVD) i refinament
amb ICP (Iterative Closest Point) sobre una superfície de referència.

WORKFLOW CLÍNIC:
    1) Marcar N landmarks aproximats sobre la vèrtebra virtual (STL).
    2) Palpar els mateixos landmarks amb el TCP (amb tolerància: el
       cirurgià no toca exactament el mateix punt).
    3) Kabsch(punts_virtuals, punts_capturats) → T_inicial.
       Aquesta transformació és aproximada perquè els punts virtuals
       són "seed" i les captures reals no hi corresponen exactament.
    4) ICP: per cada punt capturat, busca el punt més proper de la
       superfície de la vèrtebra (transformada amb T_actual). Substitueix
       les correspondències originals i re-calcula Kabsch. Repeteix fins
       que el moviment convergeixi (< 0.01 mm entre iteracions).
       Això absorbeix la imprecisió de la palpació.

Referències:
    Umeyama, S. (1991). Least-squares estimation of transformation
    parameters between two point patterns. IEEE PAMI 13(4), 376-380.
    Besl & McKay (1992). A method for registration of 3-D shapes.
    IEEE PAMI 14(2), 239-256.
"""

import math                                                                     # importem el mòdul math
import numpy as np                                                              # importem el mòdul numpy as np


def kabsch(P_src, P_dst):                                                       # definim la funció kabsch()
    """
    Transformació rígida òptima que porta P_src a P_dst.

    Entrada:
        P_src    (N, 3)  punts d'origen (p.ex. vèrtebra virtual, mm)
        P_dst    (N, 3)  punts destinació (p.ex. captures del robot, mm)

    Sortida:
        T        (4, 4)  matriu homogènia tal que:
                             p_dst ≈ (R @ p_src.T).T + t

    Requereix N ≥ 3 punts no col·lineals.
    """
    P_src = np.asarray(P_src, dtype=np.float64)                                 # P_src ← resultat de asarray()
    P_dst = np.asarray(P_dst, dtype=np.float64)                                 # P_dst ← resultat de asarray()

    if P_src.shape != P_dst.shape:                                              # si P_src.shape != P_dst.shape
        raise ValueError(f"Formes incompatibles: {P_src.shape} vs {P_dst.shape}")
    if P_src.shape[1] != 3:                                                     # si P_src.shape[1] != 3
        raise ValueError(f"Els punts han de ser (N, 3), no {P_src.shape}")
    if P_src.shape[0] < 3:                                                      # si P_src.shape[0] < 3
        raise ValueError(f"Calen ≥ 3 punts, se n'han donat {P_src.shape[0]}")

    c_src = P_src.mean(axis=0)                                                  # c_src ← resultat de mean()
    c_dst = P_dst.mean(axis=0)                                                  # c_dst ← resultat de mean()

    Q_src = P_src - c_src                                                       # assignem Q_src
    Q_dst = P_dst - c_dst                                                       # assignem Q_dst

    H = Q_src.T @ Q_dst                                                         # assignem H
    U, S, Vt = np.linalg.svd(H)                                                 # U, S, Vt ← resultat de svd()

    d = np.sign(np.linalg.det(Vt.T @ U.T))                                      # d ← resultat de sign()
    D = np.diag([1.0, 1.0, d])                                                  # D ← resultat de diag()
    R = Vt.T @ D @ U.T                                                          # assignem R

    t = c_dst - R @ c_src                                                       # assignem t

    T = np.eye(4)                                                               # T ← resultat de eye()
    T[:3, :3] = R                                                               # T[:3, :3] = R
    T[:3,  3] = t                                                               # T[:3,  3] = t
    return T                                                                    # retornem: T


def apply_T(T, points):                                                         # definim la funció apply_T()
    """Aplica una transformació homogènia 4x4 a un o més punts."""              # """Aplica una transformació homogènia 4x4 a un o més punt...
    T = np.asarray(T, dtype=np.float64)                                         # T ← resultat de asarray()
    points = np.asarray(points, dtype=np.float64)                               # points ← resultat de asarray()

    single = (points.ndim == 1)                                                 # single ← resultat de ()
    if single:                                                                  # si single
        points = points[None, :]                                                # assignem points

    N = points.shape[0]                                                         # assignem N
    homog = np.hstack([points, np.ones((N, 1))])                                # homog ← resultat de hstack()
    transformed = (T @ homog.T).T[:, :3]                                        # assignem transformed
    return transformed[0] if single else transformed                            # retornem: transformed[0] if single else transfo...


def residuals(T, P_src, P_dst):                                                 # definim la funció residuals()
    """Distàncies punt a punt després d'aplicar T a P_src."""                   # """Distàncies punt a punt després d'aplicar T a P_src."""
    P_est = apply_T(T, P_src)                                                   # P_est ← resultat de apply_T()
    return np.linalg.norm(P_est - np.asarray(P_dst), axis=1)                    # retornem: np.linalg.norm(P_est - np.asarray(P_d...


def fit_quality(T, P_src, P_dst):                                               # definim la funció fit_quality()
    """
    Estadístiques de la qualitat del registre.
    Retorna dict amb: rmse, mean, max, per_point (tot en mm).
    """
    errs = residuals(T, P_src, P_dst)                                           # errs ← resultat de residuals()
    return {                                                                    # retornem: {
        "rmse":      float(np.sqrt((errs ** 2).mean())),
        "mean":      float(errs.mean()),
        "max":       float(errs.max()),
        "per_point": errs,
    }


def inverse_T(T):                                                               # definim la funció inverse_T()
    """Inversa d'una transformació rígida homogènia."""                         # """Inversa d'una transformació rígida homogènia."""
    T = np.asarray(T, dtype=np.float64)                                         # T ← resultat de asarray()
    T_inv = np.eye(4)                                                           # T_inv ← resultat de eye()
    R = T[:3, :3]                                                               # assignem R
    t = T[:3,  3]                                                               # assignem t
    T_inv[:3, :3] = R.T                                                         # T_inv[:3, :3] = R.T
    T_inv[:3,  3] = -R.T @ t                                                    # T_inv[:3,  3] = -R.T @ t
    return T_inv                                                                # retornem: T_inv


# ==============================================================================
# ICP — Iterative Closest Point sobre superfície
# ==============================================================================

class SurfaceLocator:                                                           # definim la classe SurfaceLocator
    """
    Wrapper d'un vtkCellLocator per fer queries "punt més proper a la
    superfície" en O(log N).

    Construcció:  ~50-200 ms per una vèrtebra
    Cada query:   ~10 μs

    Ús:
        loc = SurfaceLocator(vertebra_polydata)
        closest_pt = loc.closest_point([x, y, z])
        dist_mm   = loc.distance_to([x, y, z])
    """

    def __init__(self, polydata):                                               # definim la funció __init__()
        import vtk                                                              # importem el mòdul vtk
        self._vtk = vtk                                                         # assignem self._vtk
        self._polydata = polydata                                               # assignem self._polydata

        self._locator = vtk.vtkCellLocator()                                    # self._locator ← resultat de vtkCellLocator()
        self._locator.SetDataSet(polydata)                                      # executem SetDataSet()
        self._locator.BuildLocator()                                            # executem BuildLocator()

        # Buffers reutilitzables per no crear objectes VTK a cada query
        self._closest = [0.0, 0.0, 0.0]                                         # assignem self._closest
        self._cellId  = vtk.reference(0)                                        # self._cellId ← resultat de reference()
        self._subId   = vtk.reference(0)                                        # self._subId ← resultat de reference()
        self._dist2   = vtk.reference(0.0)                                      # self._dist2 ← resultat de reference()

    def closest_point(self, pt):                                                # definim la funció closest_point()
        """Retorna el punt de la superfície més proper (com np.array shape (3,))."""  # """Retorna el punt de la superfície més proper (com np.ar...
        p = [float(pt[0]), float(pt[1]), float(pt[2])]                          # assignem p
        self._locator.FindClosestPoint(                                         # executem FindClosestPoint()
            p, self._closest, self._cellId, self._subId, self._dist2)
        return np.array(self._closest, dtype=np.float64)                        # retornem: np.array(self._closest, dtype=np.floa...

    def distance_to(self, pt):                                                  # definim la funció distance_to()
        """Distància euclidiana del punt al punt més proper de la superfície (mm)."""  # """Distància euclidiana del punt al punt més proper de la...
        p = [float(pt[0]), float(pt[1]), float(pt[2])]                          # assignem p
        self._locator.FindClosestPoint(                                         # executem FindClosestPoint()
            p, self._closest, self._cellId, self._subId, self._dist2)
        return math.sqrt(self._dist2.get())                                     # retornem: math.sqrt(self._dist2.get())

    def closest_points_batch(self, pts):                                        # definim la funció closest_points_batch()
        """Batch: (N,3) → (N,3) amb els punts més propers de la superfície."""  # """Batch: (N,3) → (N,3) amb els punts més propers de la s...
        out = np.zeros_like(pts, dtype=np.float64)                              # out ← resultat de zeros_like()
        for i, p in enumerate(pts):                                             # iterem: i, p in enumerate(pts)
            out[i] = self.closest_point(p)                                      # out[i] ← resultat de closest_point()
        return out                                                              # retornem: out


def icp_refine(P_captured, surface_locator, T_init,                             # definim la funció icp_refine()
                max_iterations=50, tolerance_mm=0.01,
                reject_ratio=None):
    """
    Refina una transformació inicial T_init amb ICP contra una superfície.

    Idea: donats N punts capturats amb el robot i una superfície de referència
    (la vèrtebra virtual), busquem la transformació que millor porta els
    punts capturats a la superfície.

        - Iteració:
            1. Aplicar T_actual als punts capturats:  P' = T @ P
            2. Per cada P'_i, trobar el punt més proper de la superfície Q_i
            3. Kabsch(P_capturats, Q_correspondents) → T_nou
            4. Convergència quan el moviment mitjà < tolerance_mm

    NOTA sobre l'orientació:
        Els punts capturats són en el frame del ROBOT.
        La superfície està en el frame de la VÈRTEBRA VIRTUAL (CT).
        T_init és la transformació virtual → robot que sortí de Kabsch.

        Per fer ICP treballem al revés: portem els punts CAPTURATS al frame
        del CT amb T_inicial^-1, busquem correspondències a la superfície CT,
        i re-fem Kabsch. Al final invertim per retornar virtual → robot.

    Parameters:
        P_captured        (N, 3) captures del robot en mm robot
        surface_locator   SurfaceLocator sobre la vèrtebra virtual (mm CT)
        T_init            transformació inicial 4×4 virtual → robot
        max_iterations    iteracions màximes
        tolerance_mm      convergència quan el desplaçament mitjà entre
                          iteracions és menor
        reject_ratio      float 0-1 o None. Si es dona, rebutja el X%
                          de correspondències més llunyanes cada iteració
                          (típic 0.1 per rebuig d'outliers)

    Retorna:
        T_final           matriu 4×4 refinada virtual → robot
        history           llista de dict amb rmse per iteració (debug)
    """
    P_captured = np.asarray(P_captured, dtype=np.float64)                       # P_captured ← resultat de asarray()
    T = np.asarray(T_init, dtype=np.float64).copy()                             # T ← resultat de asarray()

    history = []                                                                # assignem history
    prev_mean_shift = None                                                      # assignem prev_mean_shift

    for it in range(max_iterations):                                            # iterem: it in range(max_iterations)
        # 1. Portem els punts capturats al frame del CT via T^-1
        T_inv = inverse_T(T)                                                    # T_inv ← resultat de inverse_T()
        P_in_ct = apply_T(T_inv, P_captured)                                    # P_in_ct ← resultat de apply_T()

        # 2. Correspondències: per cada punt, el més proper de la superfície
        Q_correspondents = surface_locator.closest_points_batch(P_in_ct)        # Q_correspondents ← resultat de closest_points_batch()

        # 3. Distàncies punt-superfície (per rebuig d'outliers i mètriques)
        dists = np.linalg.norm(P_in_ct - Q_correspondents, axis=1)              # dists ← resultat de norm()

        # Rebuig d'outliers (opcional)
        if reject_ratio is not None and 0 < reject_ratio < 1:                   # si reject_ratio is not None and 0 < reje...
            keep_n = max(3, int(len(dists) * (1.0 - reject_ratio)))             # keep_n ← resultat de max()
            keep_idx = np.argsort(dists)[:keep_n]                               # assignem keep_idx
            src_used = Q_correspondents[keep_idx]                               # assignem src_used
            dst_used = P_captured[keep_idx]                                     # assignem dst_used
        else:                                                                   # en cas contrari
            src_used = Q_correspondents                                         # assignem src_used
            dst_used = P_captured                                               # assignem dst_used

        # 4. Nou Kabsch amb les correspondències (CT → robot)
        if len(src_used) < 3:                                                   # si len(src_used) < 3
            break                                                               # sortim del bucle
        T_new = kabsch(src_used, dst_used)                                      # T_new ← resultat de kabsch()

        # 5. Convergència: mesurem quant s'han mogut els punts
        P_new = apply_T(T_new, Q_correspondents)                                # P_new ← resultat de apply_T()
        P_old = apply_T(T,     Q_correspondents)                                # P_old ← resultat de apply_T()
        mean_shift = float(np.linalg.norm(P_new - P_old, axis=1).mean())        # mean_shift ← resultat de float()

        history.append({                                                        # executem append()
            "iter": it,
            "mean_dist_to_surface_mm": float(dists.mean()),
            "rms_dist_to_surface_mm": float(np.sqrt((dists ** 2).mean())),
            "mean_shift_mm": mean_shift,
        })

        T = T_new                                                               # assignem T

        if mean_shift < tolerance_mm:                                           # si mean_shift < tolerance_mm
            break                                                               # sortim del bucle
        if prev_mean_shift is not None and abs(prev_mean_shift - mean_shift) < 1e-6:  # si prev_mean_shift is not None and abs(p...
            break                                                               # sortim del bucle
        prev_mean_shift = mean_shift                                            # assignem prev_mean_shift

    return T, history                                                           # retornem: T, history


def surface_fit_quality(P_captured, surface_locator, T):                        # definim la funció surface_fit_quality()
    """
    Estadístiques de la qualitat del registre PUNT ↔ SUPERFÍCIE (ICP-style).
    Mesura quant lluny cauen les captures del robot de la superfície
    virtual, un cop transformades al frame CT.

    Aquesta és la mètrica correcta quan les captures no corresponen
    exactament als landmarks originals (el cas típic clínic).
    """
    P_captured = np.asarray(P_captured, dtype=np.float64)                       # P_captured ← resultat de asarray()
    T_inv = inverse_T(T)                                                        # T_inv ← resultat de inverse_T()
    P_in_ct = apply_T(T_inv, P_captured)                                        # P_in_ct ← resultat de apply_T()
    dists = np.array([surface_locator.distance_to(p) for p in P_in_ct])         # dists ← resultat de array()
    return {                                                                    # retornem: {
        "rmse":      float(np.sqrt((dists ** 2).mean())),
        "mean":      float(dists.mean()),
        "max":       float(dists.max()),
        "per_point": dists,
    }
