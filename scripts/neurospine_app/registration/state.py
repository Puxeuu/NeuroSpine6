"""
Estat del registre TAC ↔ Robot amb pipeline Kabsch + ICP.

VERSIÓ EN MEMÒRIA: cap fitxer al disc. Cada sessió es fa el registre de nou.

FLUX (dues transformacions):

    1) Kabsch inicial (rígid, punt-a-punt)
       - 8 landmarks aproximats sobre la superfície de la vèrtebra virtual
       - 8 captures del robot als "mateixos" punts físics (amb tolerància)
       - Retorna T_kabsch: aproximació inicial

    2) ICP refinat (punt-a-superfície)
       - Fa servir T_kabsch com a inicialització
       - Correspondències dinàmiques: cada captura del robot es tira contra
         la SUPERFÍCIE de la vèrtebra virtual (no contra el landmark original)
       - Convergeix a la transformació que MÉS ben ajusta les captures a la
         superfície. Absorbeix la imprecisió de la palpació.
"""

import os                                                                       # importem el mòdul os
import json                                                                     # importem el mòdul json (per persistència)
import numpy as np                                                              # importem el mòdul numpy as np
from datetime import datetime                                                   # importem datetime des de datetime

from .kabsch import (                                                           # importem ( des de .kabsch
    kabsch, fit_quality,
    icp_refine, surface_fit_quality, SurfaceLocator,
)


# ============================================================================
# CONFIGURACIÓ
# ============================================================================

MIN_POINTS = 6                                                                  # assignem MIN_POINTS

# Llindars de qualitat (mm) pel veredicte RMSE final (després d'ICP)
RMSE_EXCELLENT  = 1.0                                                           # assignem RMSE_EXCELLENT
RMSE_ACCEPTABLE = 2.0                                                           # assignem RMSE_ACCEPTABLE


class Landmark:                                                                 # definim la classe Landmark
    """Un landmark: punt virtual (mm CT) + captura opcional (mm robot)."""      # """Un landmark: punt virtual (mm CT) + captura opcional (...

    def __init__(self, idx, virtual):                                           # definim la funció __init__()
        self.idx = idx                                                          # assignem self.idx
        self.virtual = tuple(virtual)                                           # self.virtual ← resultat de tuple()
        self.robot = None                                                       # assignem self.robot

    @property                                                                   # decorador property
    def name(self):                                                             # definim la funció name()
        return f"P{self.idx + 1}"

    @property                                                                   # decorador property
    def is_captured(self):                                                      # definim la funció is_captured()
        return self.robot is not None                                           # retornem: self.robot is not None


class RegistrationState:                                                        # definim la classe RegistrationState
    """
    Estat del registre — 100% en memòria.

    Té dues transformacions:
        · T_kabsch   Kabsch inicial (rígid, punt-a-punt)
        · T          Transformació final després d'ICP (o Kabsch sol si no
                     hi ha superfície de referència)
    """

    def __init__(self):                                                         # definim la funció __init__()
        self.landmarks = []                                                     # assignem self.landmarks
        self.T_kabsch = None            # 4x4 virtual (mm CT) → robot (mm)
        self.T = None                   # 4x4 final (Kabsch + ICP)
        self.stats_kabsch = None                                                # assignem self.stats_kabsch
        self.stats_icp = None                                                   # assignem self.stats_icp
        self.icp_history = []                                                   # assignem self.icp_history
        self.meta = {}                                                          # assignem self.meta

    # ========================================================================
    # LANDMARKS
    # ========================================================================
    def add_virtual_landmark(self, x_mm, y_mm, z_mm):                           # definim la funció add_virtual_landmark()
        lm = Landmark(len(self.landmarks), (x_mm, y_mm, z_mm))                  # lm ← resultat de Landmark()
        self.landmarks.append(lm)                                               # executem append()
        self._invalidate()                                                      # executem _invalidate()
        return lm.idx                                                           # retornem: lm.idx

    def remove_landmark(self, idx):                                             # definim la funció remove_landmark()
        if 0 <= idx < len(self.landmarks):                                      # si 0 <= idx < len(self.landmarks)
            del self.landmarks[idx]                                             # eliminem: self.landmarks[idx]
            for i, lm in enumerate(self.landmarks):                             # iterem: i, lm in enumerate(self.landmarks)
                lm.idx = i                                                      # assignem lm.idx
            self._invalidate()                                                  # executem _invalidate()

    def clear_all_landmarks(self):                                              # definim la funció clear_all_landmarks()
        self.landmarks = []                                                     # assignem self.landmarks
        self._invalidate()                                                      # executem _invalidate()

    # ========================================================================
    # CAPTURES
    # ========================================================================
    def set_capture(self, idx, coords_mm):                                      # definim la funció set_capture()
        if not (0 <= idx < len(self.landmarks)):                                # si not (0 <= idx < len(self.landmarks))
            raise IndexError(f"Landmark {idx} no existeix")
        self.landmarks[idx].robot = tuple(coords_mm)                            # self.landmarks[idx].robot ← resultat de tuple()
        self._invalidate()                                                      # executem _invalidate()

    def clear_capture(self, idx):                                               # definim la funció clear_capture()
        if 0 <= idx < len(self.landmarks):                                      # si 0 <= idx < len(self.landmarks)
            self.landmarks[idx].robot = None                                    # assignem self.landmarks[idx].robot
            self._invalidate()                                                  # executem _invalidate()

    def clear_all_captures(self):                                               # definim la funció clear_all_captures()
        for lm in self.landmarks:                                               # iterem: lm in self.landmarks
            lm.robot = None                                                     # assignem lm.robot
        self._invalidate()                                                      # executem _invalidate()

    # ========================================================================
    # CONSULTES
    # ========================================================================
    def n_landmarks(self):                                                      # definim la funció n_landmarks()
        return len(self.landmarks)                                              # retornem: len(self.landmarks)

    def n_captured(self):                                                       # definim la funció n_captured()
        return sum(1 for lm in self.landmarks if lm.is_captured)                # retornem: sum(1 for lm in self.landmarks if lm....

    def can_compute(self):                                                      # definim la funció can_compute()
        return self.n_captured() >= 3                                           # retornem: self.n_captured() >= 3

    def has_enough_landmarks(self):                                             # definim la funció has_enough_landmarks()
        return len(self.landmarks) >= MIN_POINTS                                # retornem: len(self.landmarks) >= MIN_POINTS

    def has_valid_matrix(self):                                                 # definim la funció has_valid_matrix()
        return self.T is not None                                               # retornem: self.T is not None

    def captured_pairs(self):                                                   # definim la funció captured_pairs()
        idxs, p_virt, p_rob = [], [], []                                        # assignem idxs, p_virt, p_rob
        for lm in self.landmarks:                                               # iterem: lm in self.landmarks
            if lm.is_captured:                                                  # si lm.is_captured
                idxs.append(lm.idx)                                             # executem append()
                p_virt.append(lm.virtual)                                       # executem append()
                p_rob.append(lm.robot)                                          # executem append()
        return (idxs,                                                           # retornem: (idxs,
                np.array(p_virt, dtype=np.float64),
                np.array(p_rob, dtype=np.float64))

    # ========================================================================
    # CÀLCUL
    # ========================================================================
    def compute(self, vertebra_polydata=None):                                  # definim la funció compute()
        """
        Calcula el registre. Si es passa `vertebra_polydata`, s'aplica
        també el refinament ICP amb la superfície.
        """
        idxs, P_virt, P_rob = self.captured_pairs()                             # idxs, P_virt, P_rob ← resultat de captured_pairs()
        if len(idxs) < 3:                                                       # si len(idxs) < 3
            raise ValueError(f"Calen ≥ 3 parells; hi ha {len(idxs)}")

        # ---- Pas 1: Kabsch inicial (punt-a-punt) ----
        self.T_kabsch = kabsch(P_virt, P_rob)                                   # self.T_kabsch ← resultat de kabsch()
        self.stats_kabsch = fit_quality(self.T_kabsch, P_virt, P_rob)           # self.stats_kabsch ← resultat de fit_quality()

        # Per defecte, T final = Kabsch (per si no fem ICP)
        self.T = self.T_kabsch.copy()                                           # self.T ← resultat de copy()
        self.stats_icp = None                                                   # assignem self.stats_icp
        self.icp_history = []                                                   # assignem self.icp_history

        # ---- Pas 2: ICP refinat (si tenim superfície) ----
        if vertebra_polydata is not None:                                       # si vertebra_polydata is not None
            try:                                                                # intentem el bloc següent
                locator = SurfaceLocator(vertebra_polydata)                     # locator ← resultat de SurfaceLocator()
                self.T, self.icp_history = icp_refine(                          # assignem self.T, self.icp_history
                    P_captured=P_rob,
                    surface_locator=locator,
                    T_init=self.T_kabsch,
                    max_iterations=50,
                    tolerance_mm=0.005,
                    reject_ratio=0.10,      # descartem el 10% pitjor com outliers
                )
                self.stats_icp = surface_fit_quality(P_rob, locator, self.T)    # self.stats_icp ← resultat de surface_fit_quality()
            except Exception as e:                                              # capturem excepció: Exception as e
                print(f"[state] ICP ha fallat: {e}. S'usa només Kabsch.")
                import traceback; traceback.print_exc()                         # importem el mòdul traceback; traceback.print_exc()

        self.meta = {                                                           # assignem self.meta
            "computed_at":     datetime.now().isoformat(timespec="seconds"),
            "n_points":        len(idxs),
            "point_idxs":      idxs,
            "kabsch_rmse_mm":  self.stats_kabsch["rmse"],
            "icp_rmse_mm":     (self.stats_icp["rmse"]
                                 if self.stats_icp else None),
            "icp_iterations":  len(self.icp_history),
        }

        return {                                                                # retornem: {
            "kabsch_rmse":   self.stats_kabsch["rmse"],
            "icp_rmse":      (self.stats_icp["rmse"]
                              if self.stats_icp else None),
            "improvement":   ((self.stats_kabsch["rmse"] - self.stats_icp["rmse"])
                              if self.stats_icp else None),
        }

    def verdict(self):                                                          # definim la funció verdict()
        """
        Retorna (text, tone_hint) segons la millor RMSE disponible.
        Prioritza la RMSE d'ICP (punt-a-superfície), més realista.
        """
        stats = self.stats_icp or self.stats_kabsch                             # assignem stats
        if stats is None:                                                       # si stats is None
            return ("SENSE DADES", "muted")                                     # retornem: ("SENSE DADES", "muted")
        rmse = stats["rmse"]                                                    # assignem rmse
        if rmse < RMSE_EXCELLENT:                                               # si rmse < RMSE_EXCELLENT
            return ("EXCEL·LENT", "safe")                                       # retornem: ("EXCEL·LENT", "safe")
        elif rmse < RMSE_ACCEPTABLE:                                            # altrament si rmse < RMSE_ACCEPTABLE
            return ("ACCEPTABLE", "warn")                                       # retornem: ("ACCEPTABLE", "warn")
        else:                                                                   # en cas contrari
            return ("INSUFICIENT — RE-FER", "danger")                           # retornem: ("INSUFICIENT — RE-FER", "danger")

    def best_stats(self):                                                       # definim la funció best_stats()
        """Estadístiques finals: ICP si existeix, si no Kabsch."""              # """Estadístiques finals: ICP si existeix, si no Kabsch."""
        return self.stats_icp or self.stats_kabsch                              # retornem: self.stats_icp or self.stats_kabsch

    # ========================================================================
    def _invalidate(self):                                                      # definim la funció _invalidate()
        pass                                                                    # no fem res (placeholder)

    # ========================================================================
    # SERIALITZACIÓ (per MultiVertebraRegistry)
    # ========================================================================
    def to_dict(self):                                                          # definim la funció to_dict()
        """Converteix l'estat a dict JSON-serialitzable."""
        return {                                                                # retornem: dict amb tot l'estat
            "landmarks": [                                                      # llista de landmarks
                {"idx": lm.idx,
                 "virtual": list(lm.virtual),
                 "robot": list(lm.robot) if lm.robot else None}
                for lm in self.landmarks
            ],
            "T_kabsch": self.T_kabsch.tolist() if self.T_kabsch is not None else None,
            "T":        self.T.tolist()        if self.T        is not None else None,
            "stats_kabsch": self.stats_kabsch,
            "stats_icp":    self.stats_icp,
            "meta":         self.meta,
        }

    @classmethod                                                                # decorador classmethod
    def from_dict(cls, data):                                                   # definim la funció from_dict()
        """Reconstrueix un RegistrationState des d'un dict."""
        s = cls()                                                               # instància buida
        for lm_data in data.get("landmarks", []):                               # iterem els landmarks
            lm = Landmark(lm_data["idx"], lm_data["virtual"])                   # crea landmark
            if lm_data.get("robot") is not None:                                # si tenia captura
                lm.robot = tuple(lm_data["robot"])
            s.landmarks.append(lm)                                              # l'afegim
        if data.get("T_kabsch") is not None:
            s.T_kabsch = np.array(data["T_kabsch"], dtype=np.float64)
        if data.get("T") is not None:
            s.T = np.array(data["T"], dtype=np.float64)
        s.stats_kabsch = data.get("stats_kabsch")
        s.stats_icp    = data.get("stats_icp")
        s.meta         = data.get("meta", {})
        return s                                                                # retornem: s


# ============================================================================
# MULTIVERTEBRAREGISTRY — contenidor per registrar MÚLTIPLES vèrtebres
# ============================================================================
#
# Cada vèrtebra té el seu propi RegistrationState (perquè la columna NO és
# rígida: cada vèrtebra es registra per separat i té la seva pròpia matriu T).
#
# Ús típic:
#     registry = MultiVertebraRegistry("data/pacients/839201/registres.json")
#     registry.load()                            # carrega si existeix
#     registry.select("T12")                     # vèrtebra activa
#     st = registry.current()                    # estat per T12
#     # ... fer landmarks, captures, compute ...
#     registry.save()                            # desa tot al disc
#     # Vèrtebres amb registre vàlid:
#     for name in registry.names_with_valid_T():
#         T = registry[name].T                   # matriu 4x4 CT->Robot
# ============================================================================

class MultiVertebraRegistry:
    """
    Diccionari {vertebra_name: RegistrationState} amb persistència JSON.

    Convencions:
      · El nom de vèrtebra és la part DESPRÉS de "Vertebra_" del fitxer STL,
        sense extensió. Exemple: "Vertebra_T12.stl" → nom "T12".
      · El fitxer de persistència per defecte és
        <pacient_dir>/registres.json i té la forma:
            {
              "T12": { ... state.to_dict() ... },
              "L1":  { ... state.to_dict() ... }
            }
    """

    def __init__(self, json_path=None):                                         # definim la funció __init__()
        self.json_path = json_path                                              # ruta persistència (opcional)
        self._states = {}                                                       # dict nom -> RegistrationState
        self._active_name = None                                                # nom actualment seleccionat

    # ------------------------------------------------------------------
    # ACCESSORS
    # ------------------------------------------------------------------
    def names(self):                                                            # definim la funció names()
        """Noms de totes les vèrtebres en el registry."""
        return list(self._states.keys())                                        # retornem: llista de noms

    def names_with_valid_T(self):                                               # definim la funció names_with_valid_T()
        """Noms de vèrtebres amb matriu T calculada."""
        return [n for n, s in self._states.items() if s.has_valid_matrix()]     # retornem: noms amb T

    def __contains__(self, name):                                               # definim operador in
        return name in self._states                                             # retornem: True si existeix

    def __getitem__(self, name):                                                # definim accés per clau
        return self._states[name]                                               # retornem: estat (o KeyError)

    def ensure(self, name):                                                     # definim la funció ensure()
        """Retorna l'estat de 'name', creant-lo buit si no existeix."""
        if name not in self._states:                                            # si no existeix
            self._states[name] = RegistrationState()                            # el creem
        return self._states[name]                                               # retornem: l'estat

    def active_name(self):                                                      # definim la funció active_name()
        return self._active_name                                                # retornem: nom actiu

    def select(self, name):                                                     # definim la funció select()
        """Activa una vèrtebra. La crea si no existia encara."""
        self.ensure(name)                                                       # garantim que existeixi
        self._active_name = name                                                # fixem com activa
        return self._states[name]                                               # retornem: l'estat actiu

    def current(self):                                                          # definim la funció current()
        """Estat de la vèrtebra activa (None si cap)."""
        if self._active_name is None:                                           # si no hi ha activa
            return None                                                         # retornem: None
        return self._states.get(self._active_name)                              # retornem: estat o None

    def remove(self, name):                                                     # definim la funció remove()
        """Elimina la vèrtebra del registry."""
        self._states.pop(name, None)                                            # treu si existia
        if self._active_name == name:                                           # si era l'activa
            self._active_name = None                                            # reset

    # ------------------------------------------------------------------
    # PERSISTÈNCIA
    # ------------------------------------------------------------------
    def to_dict(self):                                                          # definim la funció to_dict()
        """Serialitza tot el registry a dict JSON-serialitzable."""
        return {                                                                # retornem: {nom: state_dict}
            name: state.to_dict()
            for name, state in self._states.items()
        }

    def from_dict(self, data):                                                  # definim la funció from_dict()
        """Reemplaça l'estat actual pel que ve del dict."""
        self._states = {                                                        # reconstruïm tots els estats
            name: RegistrationState.from_dict(state_data)
            for name, state_data in (data or {}).items()
        }
        # Si no hi havia activa o la que hi havia ja no existeix, no fem res.
        if self._active_name not in self._states:                               # si l'activa no existeix
            self._active_name = None                                            # reset

    def save(self, path=None):                                                  # definim la funció save()
        """Desa el registry al disc. Usa self.json_path si path=None."""
        target = path or self.json_path                                         # ruta final
        if not target:                                                          # si no hi ha ruta
            raise ValueError("Cal un path per desar el registry")
        os.makedirs(os.path.dirname(target), exist_ok=True)                     # creem carpeta si cal
        with open(target, "w", encoding="utf-8") as f:                          # obrim per escriure
            json.dump(self.to_dict(), f, indent=2, ensure_ascii=False)
        return target                                                           # retornem: ruta on hem desat

    def load(self, path=None):                                                  # definim la funció load()
        """Carrega el registry del disc (si existeix). Silent si no hi és."""
        target = path or self.json_path                                         # ruta
        if not target or not os.path.exists(target):                            # si no existeix
            return False                                                        # retornem: False (res carregat)
        try:                                                                    # intentem llegir
            with open(target, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.from_dict(data)                                                # reemplacem l'estat
            return True                                                         # retornem: True (carregat)
        except Exception as e:                                                  # capturem errors
            print(f"[MultiVertebraRegistry] error carregant {target}: {e}")
            return False                                                        # retornem: False
