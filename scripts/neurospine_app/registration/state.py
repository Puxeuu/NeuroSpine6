import os
import json
import numpy as np
from datetime import datetime

from .kabsch import (
    kabsch, fit_quality,
    icp_refine, surface_fit_quality, SurfaceLocator,
)


MIN_POINTS = 6

RMSE_EXCELLENT  = 1.0
RMSE_ACCEPTABLE = 2.0


class Landmark:
    def __init__(self, idx, virtual):
        self.idx = idx
        self.virtual = tuple(virtual)
        self.robot = None

    @property
    def name(self):
        return f"P{self.idx + 1}"

    @property
    def is_captured(self):
        return self.robot is not None


class RegistrationState:
    def __init__(self):
        self.landmarks = []
        self.T_kabsch = None
        self.T = None
        self.stats_kabsch = None
        self.stats_icp = None
        self.icp_history = []
        self.meta = {}

    def add_virtual_landmark(self, x_mm, y_mm, z_mm):
        lm = Landmark(len(self.landmarks), (x_mm, y_mm, z_mm))
        self.landmarks.append(lm)
        self._invalidate()
        return lm.idx

    def remove_landmark(self, idx):
        if 0 <= idx < len(self.landmarks):
            del self.landmarks[idx]
            for i, lm in enumerate(self.landmarks):
                lm.idx = i
            self._invalidate()

    def clear_all_landmarks(self):
        self.landmarks = []
        self._invalidate()

    def set_capture(self, idx, coords_mm):
        if not (0 <= idx < len(self.landmarks)):
            raise IndexError(f"Landmark {idx} no existeix")
        self.landmarks[idx].robot = tuple(coords_mm)
        self._invalidate()

    def clear_capture(self, idx):
        if 0 <= idx < len(self.landmarks):
            self.landmarks[idx].robot = None
            self._invalidate()

    def clear_all_captures(self):
        for lm in self.landmarks:
            lm.robot = None
        self._invalidate()

    def n_landmarks(self):
        return len(self.landmarks)

    def n_captured(self):
        return sum(1 for lm in self.landmarks if lm.is_captured)

    def can_compute(self):
        return self.n_captured() >= 3

    def has_enough_landmarks(self):
        return len(self.landmarks) >= MIN_POINTS

    def has_valid_matrix(self):
        return self.T is not None

    def captured_pairs(self):
        idxs, p_virt, p_rob = [], [], []
        for lm in self.landmarks:
            if lm.is_captured:
                idxs.append(lm.idx)
                p_virt.append(lm.virtual)
                p_rob.append(lm.robot)
        return (idxs,
                np.array(p_virt, dtype=np.float64),
                np.array(p_rob, dtype=np.float64))

    def compute(self, vertebra_polydata=None):
        idxs, P_virt, P_rob = self.captured_pairs()
        if len(idxs) < 3:
            raise ValueError(f"Calen ≥ 3 parells; hi ha {len(idxs)}")

        self.T_kabsch = kabsch(P_virt, P_rob)
        self.stats_kabsch = fit_quality(self.T_kabsch, P_virt, P_rob)

        self.T = self.T_kabsch.copy()
        self.stats_icp = None
        self.icp_history = []

        if vertebra_polydata is not None:
            try:
                locator = SurfaceLocator(vertebra_polydata)
                self.T, self.icp_history = icp_refine(
                    P_captured=P_rob,
                    surface_locator=locator,
                    T_init=self.T_kabsch,
                    max_iterations=50,
                    tolerance_mm=0.005,
                    reject_ratio=0.10,
                )
                self.stats_icp = surface_fit_quality(P_rob, locator, self.T)
            except Exception as e:
                print(f"[state] ICP ha fallat: {e}. S'usa només Kabsch.")
                import traceback; traceback.print_exc()

        self.meta = {
            "computed_at":     datetime.now().isoformat(timespec="seconds"),
            "n_points":        len(idxs),
            "point_idxs":      idxs,
            "kabsch_rmse_mm":  self.stats_kabsch["rmse"],
            "icp_rmse_mm":     (self.stats_icp["rmse"]
                                 if self.stats_icp else None),
            "icp_iterations":  len(self.icp_history),
        }

        return {
            "kabsch_rmse":   self.stats_kabsch["rmse"],
            "icp_rmse":      (self.stats_icp["rmse"]
                              if self.stats_icp else None),
            "improvement":   ((self.stats_kabsch["rmse"] - self.stats_icp["rmse"])
                              if self.stats_icp else None),
        }

    def verdict(self):
        stats = self.stats_icp or self.stats_kabsch
        if stats is None:
            return ("SENSE DADES", "muted")
        rmse = stats["rmse"]
        if rmse < RMSE_EXCELLENT:
            return ("EXCEL·LENT", "safe")
        elif rmse < RMSE_ACCEPTABLE:
            return ("ACCEPTABLE", "warn")
        else:
            return ("INSUFICIENT — RE-FER", "danger")

    def best_stats(self):
        return self.stats_icp or self.stats_kabsch

    def _invalidate(self):
        pass

    def to_dict(self):
        return {
            "landmarks": [
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

    @classmethod
    def from_dict(cls, data):
        s = cls()
        for lm_data in data.get("landmarks", []):
            lm = Landmark(lm_data["idx"], lm_data["virtual"])
            if lm_data.get("robot") is not None:
                lm.robot = tuple(lm_data["robot"])
            s.landmarks.append(lm)
        if data.get("T_kabsch") is not None:
            s.T_kabsch = np.array(data["T_kabsch"], dtype=np.float64)
        if data.get("T") is not None:
            s.T = np.array(data["T"], dtype=np.float64)
        s.stats_kabsch = data.get("stats_kabsch")
        s.stats_icp    = data.get("stats_icp")
        s.meta         = data.get("meta", {})
        return s


class MultiVertebraRegistry:
    def __init__(self, json_path=None):
        self.json_path = json_path
        self._states = {}
        self._active_name = None

    def names(self):
        return list(self._states.keys())

    def names_with_valid_T(self):
        return [n for n, s in self._states.items() if s.has_valid_matrix()]

    def __contains__(self, name):
        return name in self._states

    def __getitem__(self, name):
        return self._states[name]

    def ensure(self, name):
        if name not in self._states:
            self._states[name] = RegistrationState()
        return self._states[name]

    def active_name(self):
        return self._active_name

    def select(self, name):
        self.ensure(name)
        self._active_name = name
        return self._states[name]

    def current(self):
        if self._active_name is None:
            return None
        return self._states.get(self._active_name)

    def remove(self, name):
        self._states.pop(name, None)
        if self._active_name == name:
            self._active_name = None

    def to_dict(self):
        return {
            name: state.to_dict()
            for name, state in self._states.items()
        }

    def from_dict(self, data):
        self._states = {
            name: RegistrationState.from_dict(state_data)
            for name, state_data in (data or {}).items()
        }
        if self._active_name not in self._states:
            self._active_name = None

    def save(self, path=None):
        target = path or self.json_path
        if not target:
            raise ValueError("Cal un path per desar el registry")
        os.makedirs(os.path.dirname(target), exist_ok=True)
        with open(target, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, indent=2, ensure_ascii=False)
        return target

    def load(self, path=None):
        target = path or self.json_path
        if not target or not os.path.exists(target):
            return False
        try:
            with open(target, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.from_dict(data)
            return True
        except Exception as e:
            print(f"[MultiVertebraRegistry] error carregant {target}: {e}")
            return False

