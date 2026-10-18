"""
Anàlisi d'ancoratge d'un cargol planificat, a partir dels HU REALS del TAC.

El mapa de porositat de colors és subjectiu (cal mirar-lo i interpretar-lo).
Aquí, en canvi, es MOSTREJA el TAC exactament on anirà el cargol i se'n
treuen números objectius:

    1. DENSITAT D'ANCORATGE   HU mitjans de l'os que mosseguen les estries
                              (anell entre el nucli i el diàmetre exterior),
                              i el PERFIL al llarg del cargol.
    2. PERFORACIÓ             just fora de la paret del cargol: si un sector
                              cau a teixit tou, el cargol surt de l'os en
                              aquella profunditat (i es diu si va cap al canal).
    3. MEDUL·LA               distància mínima de la superfície del cargol.
    4. Ø MÀXIM                el corredor ossi més estret (istme del pedicle)
                              i el diàmetre estàndard més gran que hi cap.
    5. OPTIMITZACIÓ           amb el punt d'entrada fix, prova inclinacions de
                              fins a ±max_deg i proposa la que maximitza la
                              densitat sense perforar i lluny de la medul·la.

Tot en mm, al marc físic del TAC: p_mm = origin + índex · spacing.
Mòdul pur (numpy/scipy + vtk opcional per a la medul·la): es pot provar sol.

LLINDARS ORIENTATIUS (no validats clínicament, configurables):
    HU_SOFT  per sota → fora de l'os (teixit tou, greix, LCR).
    HU_LOW / HU_OK: referència de densitat trabecular (Pickhardt et al.,
    2013: HU de L1 ≤ 110 → probable osteoporosi; ≥ 160 → normal).
"""

import numpy as np                                                              # importem el mòdul numpy as np
from scipy.ndimage import map_coordinates                                       # interpolació trilineal al volum

HU_SOFT = 80.0                                                                  # per sota: fora de l'os
HU_LOW  = 110.0                                                                 # densitat d'ancoratge baixa (orientatiu)
HU_OK   = 160.0                                                                 # densitat d'ancoratge bona (orientatiu)

STEP_MM        = 0.5                                                            # pas al llarg del cargol
N_THETA        = 24                                                             # sectors al voltant de l'eix
ENTRY_SKIP_MM  = 2.0                                                            # a l'entrada el cargol travessa la superfície: no és perforació
WALL_GAP_MM    = 0.75                                                           # on mirem "just fora" de la paret
BREACH_SECTORS = 2                                                              # sectors seguits fora de l'os per comptar perforació
MED_MIN_MM     = 2.0                                                            # medul·la: vermell per sota
MED_OK_MM      = 4.0                                                            # medul·la: groc per sota
DIAM_MARGIN_MM = 0.5                                                            # marge a cada banda per al Ø recomanat
CORRIDOR_MAX_MM = 8.0                                                           # fins on busquem la vora de l'os
STD_DIAMETERS  = (4.5, 5.5, 6.5, 7.5)                                           # diàmetres disponibles


class CTSampler:                                                                # definim la classe CTSampler
    """Mostreja HU del volum (z, y, x) en punts en mm del marc físic."""

    def __init__(self, vol_zyx, spacing_xyz, origin_xyz):                       # definim la funció __init__()
        self.vol = vol_zyx                                                      # assignem self.vol
        self.sp = np.asarray(spacing_xyz, dtype=float)                          # assignem self.sp
        self.org = np.asarray(origin_xyz, dtype=float)                          # assignem self.org

    def hu(self, pts_mm):                                                       # definim la funció hu()
        pts = np.asarray(pts_mm, dtype=float).reshape(-1, 3)                    # pts ← resultat de reshape()
        idx = (pts - self.org) / self.sp                                        # mm → índex (x, y, z)
        return map_coordinates(self.vol, [idx[:, 2], idx[:, 1], idx[:, 0]],     # retornem: HU interpolats
                               order=1, mode="nearest").astype(float)


class MedullaDistance:                                                          # definim la classe MedullaDistance
    """Distància de punts a la superfície de la medul·la (vtkStaticCellLocator, construït un cop)."""

    def __init__(self, polydata):                                               # definim la funció __init__()
        import vtk                                                              # importem el mòdul vtk
        self._loc = vtk.vtkStaticCellLocator()                                  # self._loc ← resultat de vtkStaticCellLocator()
        self._loc.SetDataSet(polydata)                                          # executem SetDataSet()
        self._loc.BuildLocator()                                                # executem BuildLocator()
        self._vtk = vtk                                                         # assignem self._vtk

    def closest(self, pts_mm):                                                  # definim la funció closest()
        """(distàncies, punts més propers) per a cada punt."""
        vtk = self._vtk                                                         # assignem vtk
        cp = [0.0, 0.0, 0.0]; cid = vtk.reference(0); sid = vtk.reference(0); d2 = vtk.reference(0.0)
        dists, closest = [], []                                                 # assignem dists, closest
        for p in np.asarray(pts_mm, dtype=float).reshape(-1, 3):                # iterem: p in pts_mm
            self._loc.FindClosestPoint(p.tolist(), cp, cid, sid, d2)            # executem FindClosestPoint()
            dists.append(float(d2) ** 0.5)                                      # executem append()
            closest.append(list(cp))                                            # executem append()
        return np.array(dists), np.array(closest)                               # retornem: dists, closest


def perpendicular_frame(d):                                                     # definim la funció perpendicular_frame()
    """Dos vectors unitaris perpendiculars a d (i entre ells)."""
    d = np.asarray(d, dtype=float)                                              # d ← resultat de asarray()
    ref = np.array([0.0, 0.0, 1.0]) if abs(d[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(d, ref); u /= np.linalg.norm(u)                                # u ← resultat de cross()
    v = np.cross(d, u)                                                          # v ← resultat de cross()
    return u, v                                                                 # retornem: u, v


def _ring_points(p0, d, u, v, t, radius, theta):                                # definim la funció _ring_points()
    """Punts (T, N, 3): a cada profunditat t, un anell de radi `radius`."""
    axis = p0[None, :] + t[:, None] * d[None, :]                                # (T, 3)
    ring = np.cos(theta)[:, None] * u[None, :] + np.sin(theta)[:, None] * v[None, :]  # (N, 3)
    return axis[:, None, :] + radius * ring[None, :, :]                         # retornem: (T, N, 3)


def _runs(mask):                                                                # definim la funció _runs()
    """Trams seguits de True en un vector booleà → llista de (inici, fi) inclusius."""
    runs, start = [], None                                                      # assignem runs, start
    for i, m in enumerate(mask):                                                # iterem: i, m in enumerate(mask)
        if m and start is None:                                                 # si m and start is None
            start = i                                                           # assignem start
        elif not m and start is not None:                                       # altrament si not m and start is not None
            runs.append((start, i - 1)); start = None                           # executem append()
    if start is not None:                                                       # si start is not None
        runs.append((start, len(mask) - 1))                                     # executem append()
    return runs                                                                 # retornem: runs


def _circular_run(mask_row, k):                                                 # definim la funció _circular_run()
    """True si hi ha k sectors seguits True (circularment)."""
    if k <= 1:                                                                  # si k <= 1
        return bool(mask_row.any())                                             # retornem: bool
    m = np.concatenate([mask_row, mask_row[:k - 1]])                            # m ← resultat de concatenate()
    c = np.convolve(m.astype(int), np.ones(k, int), mode="valid")               # c ← resultat de convolve()
    return bool((c >= k).any())                                                 # retornem: bool


def analyze_screw(sampler, p0_mm, p1_mm, diam_mm, medulla=None, light=False):   # definim la funció analyze_screw()
    """
    Analitza un cargol de p0 (entrada) a p1 (punta). Retorna un dict:

        length_mm, anchor_hu, profile (t_mm, hu), breaches [(t0, t1, cap_al_canal)],
        breach_mm (llargada total perforada), med_dist_mm, med_at_mm,
        corridor (t_mm, radi_mm), isthmus_mm, isthmus_r_mm, max_diam_mm,
        recommended_diam, verdict ("ok"/"warn"/"danger"), notes [str]

    light=True: només el necessari per a l'optimitzador (sense corredor).
    """
    p0 = np.asarray(p0_mm, dtype=float); p1 = np.asarray(p1_mm, dtype=float)    # assignem p0
    axis = p1 - p0                                                              # assignem axis
    L = float(np.linalg.norm(axis))                                             # L ← resultat de float()
    if L < 1.0:                                                                 # massa curt
        return None                                                             # retornem: None
    d = axis / L                                                                # assignem d
    u, v = perpendicular_frame(d)                                               # u, v ← resultat de perpendicular_frame()
    t = np.arange(0.0, L + 1e-9, STEP_MM)                                       # t ← resultat de arange()
    theta = np.linspace(0.0, 2.0 * np.pi, N_THETA, endpoint=False)              # theta ← resultat de linspace()
    R = diam_mm / 2.0                                                           # assignem R

    # 1. Densitat a la zona de les estries (entre el nucli ≈0.7·R i R)
    radii = (0.75 * R, 0.95 * R)                                                # assignem radii
    hu_thread = np.stack([sampler.hu(_ring_points(p0, d, u, v, t, r, theta)).reshape(len(t), -1)
                          for r in radii], axis=1).reshape(len(t), -1)          # (T, 2N)
    profile = hu_thread.mean(axis=1)                                            # HU mitjà a cada profunditat
    inside = t >= ENTRY_SKIP_MM                                                 # dins l'os (després de l'entrada)
    anchor_hu = float(hu_thread[inside].mean()) if inside.any() else float(hu_thread.mean())

    # 2. Perforació: just fora de la paret
    wall = sampler.hu(_ring_points(p0, d, u, v, t, R + WALL_GAP_MM, theta)).reshape(len(t), -1)
    out = wall < HU_SOFT                                                        # (T, N) sectors fora de l'os
    breach_t = np.array([inside[i] and _circular_run(out[i], BREACH_SECTORS) for i in range(len(t))])
    runs = _runs(breach_t)                                                      # runs ← resultat de _runs()

    res = {"length_mm": L, "anchor_hu": anchor_hu, "profile": (t, profile),
           "breaches": [], "breach_mm": 0.0, "med_dist_mm": None, "med_at_mm": None}

    # 3. Medul·la: distància de la superfície del cargol
    med_dir = None                                                              # assignem med_dir
    if medulla is not None:                                                     # si medulla is not None
        axis_pts = p0[None, :] + t[:, None] * d[None, :]                        # axis_pts ← resultat de +()
        dists, closest = medulla.closest(axis_pts)                              # dists, closest ← resultat de closest()
        surf = dists - R                                                        # de la superfície del cargol
        k = int(np.argmin(surf))                                                # k ← resultat de int()
        res["med_dist_mm"] = float(surf[k])                                     # assignem res["med_dist_mm"]
        res["med_at_mm"] = float(t[k])                                          # assignem res["med_at_mm"]
        med_dir = closest - axis_pts                                            # cap al canal, a cada profunditat
        med_dir /= np.maximum(np.linalg.norm(med_dir, axis=1, keepdims=True), 1e-9)

    ring_dirs = np.cos(theta)[:, None] * u[None, :] + np.sin(theta)[:, None] * v[None, :]
    for a, b in runs:                                                           # iterem: a, b in runs
        toward = False                                                          # assignem toward
        if med_dir is not None:                                                 # la perforació mira cap al canal?
            for i in range(a, b + 1):                                           # iterem: i in range(a, b + 1)
                if out[i].any():                                                # si out[i].any()
                    bdir = ring_dirs[out[i]].mean(axis=0)                       # direcció mitjana dels sectors fora
                    n = np.linalg.norm(bdir)                                    # n ← resultat de norm()
                    if n > 1e-6 and np.dot(bdir / n, med_dir[i]) > 0.5:         # si n > 1e-6 and ...
                        toward = True; break                                    # assignem toward
        res["breaches"].append((float(t[a]), float(t[b]), toward))              # executem append()
        res["breach_mm"] += float(t[b] - t[a] + STEP_MM)                        # sumem a res["breach_mm"]

    if light:                                                                   # optimitzador: prou
        return res                                                              # retornem: res

    # 4. Corredor ossi: a cada profunditat i sector, fins on hi ha os
    rr = np.arange(0.5, CORRIDOR_MAX_MM + 1e-9, 0.25)                           # rr ← resultat de arange()
    tc = t[inside & (t <= L - 1.0)]                                             # profunditats (sense la punta)
    corridor = np.full(len(tc), np.nan)                                         # corridor ← resultat de full()
    if len(tc):                                                                 # si len(tc)
        pts = (p0[None, None, None, :] + tc[:, None, None, None] * d[None, None, None, :]
               + rr[None, None, :, None] * (np.cos(theta)[None, :, None, None] * u
                                            + np.sin(theta)[None, :, None, None] * v))
        h = sampler.hu(pts).reshape(len(tc), len(theta), len(rr))               # (T, N, R)
        soft = h < HU_SOFT                                                      # assignem soft
        first = np.where(soft.any(axis=2), soft.argmax(axis=2), len(rr) - 1)    # primer radi fora de l'os
        rb = rr[first]                                                          # (T, N) radi de la vora
        corridor = rb.min(axis=1)                                               # sector més estret a cada t
    res["corridor"] = (tc, corridor)                                            # assignem res["corridor"]
    if len(tc) and np.isfinite(corridor).any():                                 # istme = on el corredor és més estret
        j = int(np.nanargmin(corridor))                                         # j ← resultat de int()
        res["isthmus_mm"], res["isthmus_r_mm"] = float(tc[j]), float(corridor[j])
        res["max_diam_mm"] = max(0.0, 2.0 * (corridor[j] - DIAM_MARGIN_MM))     # assignem res["max_diam_mm"]
        fit = [dd for dd in STD_DIAMETERS if dd <= res["max_diam_mm"] + 1e-9]   # fit ← resultat de list()
        res["recommended_diam"] = fit[-1] if fit else None                      # assignem res["recommended_diam"]
    else:                                                                       # en cas contrari
        res["isthmus_mm"] = res["isthmus_r_mm"] = res["max_diam_mm"] = None     # assignem
        res["recommended_diam"] = None                                          # assignem res["recommended_diam"]

    # 5. Veredicte
    notes, level = [], 0                                                        # assignem notes, level
    if res["breaches"]:                                                         # si res["breaches"]
        canal = any(b[2] for b in res["breaches"])                              # canal ← resultat de any()
        a, b = res["breaches"][0][0], res["breaches"][-1][1]                    # assignem a, b
        notes.append(f"Surt de l'os entre {a:.0f} i {b:.0f} mm" + (" · CAP AL CANAL" if canal else ""))
        level = 2 if (canal or res["breach_mm"] >= 2.0) else max(level, 1)      # assignem level
    if res["med_dist_mm"] is not None:                                          # si hi ha medul·la
        if res["med_dist_mm"] <= 0:                                             # el cargol toca el canal
            notes.append(f"Toca el canal medul·lar (a {res['med_at_mm']:.0f} mm)"); level = 2
        elif res["med_dist_mm"] < MED_MIN_MM:                                   # si massa a prop
            notes.append(f"Medul·la a {res['med_dist_mm']:.1f} mm"); level = 2
        elif res["med_dist_mm"] < MED_OK_MM:                                    # altrament si a prop
            notes.append(f"Medul·la a {res['med_dist_mm']:.1f} mm"); level = max(level, 1)
    if anchor_hu < HU_LOW:                                                      # si densitat baixa
        notes.append("Densitat d'ancoratge baixa"); level = max(level, 1)
    if res["max_diam_mm"] is not None and diam_mm > res["max_diam_mm"] + 1e-9:  # el Ø no hi cap amb marge
        notes.append(f"Ø {diam_mm:.1f} massa gran per a l'istme"); level = max(level, 1)
    res["notes"] = notes                                                        # assignem res["notes"]
    res["verdict"] = ("ok", "warn", "danger")[level]                            # assignem res["verdict"]
    return res                                                                  # retornem: res


def _rotate(vec, axis, ang):                                                    # definim la funció _rotate()
    """Rodrigues: gira vec un angle ang (rad) al voltant d'axis (unitari)."""
    c, s = np.cos(ang), np.sin(ang)                                             # assignem c, s
    return vec * c + np.cross(axis, vec) * s + axis * np.dot(axis, vec) * (1 - c)


def optimize_trajectory(sampler, p0_mm, p1_mm, diam_mm, medulla=None,          # definim la funció optimize_trajectory()
                        max_deg=6.0, step_deg=1.5):
    """
    Entrada fixa, llargada fixa: prova direccions dins un con de ±max_deg i
    tria la de més densitat d'ancoratge que:
        · no perfora (més enllà de l'entrada)
        · queda a ≥ MED_OK_MM de la medul·la (o, si l'actual ja no hi arriba,
          almenys no s'hi acosta més)
    Petita penalització per l'angle, per no canviar de trajectòria per res.
    Retorna (p1_nou, anàlisi_nova, millora_%) o None si no hi ha res millor.
    """
    p0 = np.asarray(p0_mm, dtype=float); p1 = np.asarray(p1_mm, dtype=float)    # assignem p0
    L = float(np.linalg.norm(p1 - p0)); d0 = (p1 - p0) / L                      # assignem L
    u, v = perpendicular_frame(d0)                                              # u, v ← resultat de perpendicular_frame()
    base = analyze_screw(sampler, p0, p1, diam_mm, medulla, light=True)         # base ← resultat de analyze_screw()
    if base is None:                                                            # si base is None
        return None                                                             # retornem: None
    med_floor = MED_OK_MM                                                       # assignem med_floor
    if base["med_dist_mm"] is not None and base["med_dist_mm"] < MED_OK_MM:     # l'actual ja és a prop
        med_floor = base["med_dist_mm"]                                         # assignem med_floor

    best = None                                                                 # assignem best
    steps = np.arange(-max_deg, max_deg + 1e-9, step_deg)                       # steps ← resultat de arange()
    for a in steps:                                                             # iterem: a in steps
        for b in steps:                                                         # iterem: b in steps
            ang = np.hypot(a, b)                                                # ang ← resultat de hypot()
            if ang > max_deg + 1e-9 or ang < 1e-9:                              # dins del con i diferent de l'actual
                continue                                                        # passem a la següent iteració
            dd = _rotate(_rotate(d0, u, np.radians(a)), v, np.radians(b))       # dd ← resultat de _rotate()
            r = analyze_screw(sampler, p0, p0 + L * dd, diam_mm, medulla, light=True)
            if r is None or r["breaches"]:                                      # perfora
                continue                                                        # passem a la següent iteració
            if r["med_dist_mm"] is not None and r["med_dist_mm"] < med_floor - 1e-6:
                continue                                                        # massa a prop de la medul·la
            # densitat − 3 HU per grau (no canviar per res) + 40 HU per mm
            # allunyat de la medul·la fins a MED_OK_MM (la seguretat mana)
            med_term = 0.0                                                      # assignem med_term
            if r["med_dist_mm"] is not None:                                    # si hi ha medul·la
                med_term = 40.0 * min(r["med_dist_mm"], MED_OK_MM)              # assignem med_term
            score = r["anchor_hu"] - 3.0 * ang + med_term                       # score ← resultat de +()
            if best is None or score > best[0]:                                 # si és la millor
                best = (score, p0 + L * dd, ang)                                # assignem best

    if best is None:                                                            # cap alternativa vàlida
        return None                                                             # retornem: None
    new = analyze_screw(sampler, p0, best[1], diam_mm, medulla)                 # anàlisi completa de la proposta
    gain = (new["anchor_hu"] - base["anchor_hu"]) / max(abs(base["anchor_hu"]), 1.0) * 100.0
    safer = (base["med_dist_mm"] is not None and base["med_dist_mm"] < MED_OK_MM
             and new["med_dist_mm"] is not None
             and new["med_dist_mm"] >= base["med_dist_mm"] + 0.5)               # s'allunya del canal
    better = (bool(base["breaches"]) and not new["breaches"]) or safer or gain >= 3.0  # val la pena?
    if not better:                                                              # si no millora prou
        return None                                                             # retornem: None
    return best[1], new, gain, best[2]                                          # retornem: p1, anàlisi, %, angle
