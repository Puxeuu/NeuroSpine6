"""
Fa de pont entre el viewer (que treballa en radians) i el MotorBus (que treballa en counts d'encoder).

"""

import numpy as np
from typing import Optional
from src.hardware.sitodrive import (
    MotorBus, Mode, encode_position, encode_profile_position,
    encode_mode_selection,
)
from src.hardware.motor_config import MotorConfig
from src.hardware.motor_converter import q_to_motor_counts, motor_counts_to_q


# ==============================================================================
# MOTORCONTROLLER
# ==============================================================================

class MotorController:

    def __init__(self, config: MotorConfig, port: Optional[str] = None):        # Rep la config del YAML i opcionalment un port. Optional[str] = None vol dir "pot ser string o None, per defecte None".
        self.config = config
        self.port = port
        self.bus: Optional[MotorBus] = None
        self._connected = False                                                 # assignem self._connected
        self._mode: Optional[Mode] = None                                       # self._mode: Optional[Mode] = None

        # Llista de motor_id a EXCLOURE del send_q. S'omple quan algú altre
        # pren el control d'un motor (ex: J6 en CURRENT_MODE durant torque,
        # o J6 en RESET durant IK5). Si send_q enviés encode_position a un
        # motor exclòs, cada paquet (que porta cmd=POSITION_MODE) treuria
        # el motor del seu mode actual → caos. Vegeu send_q() més avall.
        self._auto_exclude_motor_ids: list = []                                 # per defecte, cap exclusió



    def connect(self) -> None:                                                  # Obre el port sèrie (o inicia el simulador si port=None)
        if self._connected:                                                     # si ja estem connectats, no fem res. Evita duplicar connexions.
            return
        self.bus = MotorBus(motor_ids=self.config.motor_ids, port=self.port, baudrate=self.config.baudrate,)        # Internament, el MotorBus decidirà si fa servir el daemon (si port != None) o el simulador (si port = None).
        self._connected = True



    def disconnect(self) -> None:                                               # Tanca el port sèrie
        if not self._connected:
            return
        if self.bus is not None:
            self.bus.close()                                                    # executem close()
        self.bus = None                                                         # Netejem l'estat intern.
        self._connected = False
        self._mode = None

    @property
    def is_connected(self) -> bool:                                             # Permet llegir l'estat sense permetre modificar-lo.
        return self._connected




    def enable_position_control(self) -> None:
        """
        Envia select_mode(POSITION_MODE) al bus, RESPECTANT els motors
        exclosos a _auto_exclude_motor_ids. Si no hi ha exclusió, fa el
        camí ràpid de sempre (bus.select_mode envia a tots).

        Per què la distinció? Perquè encode_mode_selection és un paquet
        "canvi de mode sense target" i molts firmwares, en rebre'l,
        reinicien el seu target intern (sovint a 0 o a la posició zero).
        Si el J6 està en POSITION_MODE amb un target correcte (fixat pel
        _ensure_j6_included via encode_position atòmic), aquest paquet
        li faria perdre aquell target i començaria a girar enrere.
        """
        self._require_connected()                                               # requereix connexió
        exclude = set(self._auto_exclude_motor_ids or ())                       # motors a saltar
        if not exclude:                                                         # cap exclusió → camí ràpid
            self.bus.select_mode(Mode.POSITION_MODE)
        else:                                                                   # amb exclusió → motor a motor
            for joint in self.config.joints:                                    # iterem els joints
                if joint.motor_id in exclude:                                   # si aquest està exclòs
                    continue                                                    # el saltem (no li toquem el mode)
                pkt = encode_mode_selection(                                    # paquet mode_selection
                    joint.motor_id, Mode.POSITION_MODE,
                    fb_cycle1=10, fb_cycle2=10, fb_cycle3=0,
                )
                if self.bus._daemon is not None:                                # bus amb daemon
                    self.bus._daemon.send({"cmd": "raw_bytes", "bytes": list(pkt)})
                else:                                                           # bus amb simulador
                    self.bus._sim.feed_command(pkt)
        self._mode = Mode.POSITION_MODE                                         # recordem el mode del bus

    def enable_position_control_at(self, q_rad: np.ndarray) -> None:
        """
        Activa POSITION_MODE amb el target = q_rad, SENSE finestra.

        El firmware recorda l'últim target de posició; en rebre
        mode_selection(POSITION) el reprèn, i si el motor s'ha mogut des
        d'aleshores (broca, hand-guide, braç mogut en RESET) aniria cap allà.
        Per això enviem el target just ABANS (per si el firmware el guarda) i
        just DESPRÉS del mode_selection: al bus són pocs mil·lisegons.
        Els motors exclosos (_auto_exclude_motor_ids) no es toquen.
        """
        self._require_connected()                                               # requereix connexió
        self._mode = Mode.POSITION_MODE                                         # send_q ho necessita
        try:                                                                    # target previ (pot ser ignorat en RESET)
            self.send_q(q_rad)
        except Exception:
            pass
        self.enable_position_control()                                          # mode_selection(POSITION)
        for _ in range(3):                                                      # target enganxat darrere
            self.send_q(q_rad)

    def enable_profile_position_control(self) -> None:
        """Com enable_position_control, però amb PROFILE_POSITION_MODE (amb rampa)."""
        self._require_connected()                                               # requereix connexió
        exclude = set(self._auto_exclude_motor_ids or ())                       # motors a saltar
        if not exclude:                                                         # cap exclusió → camí ràpid
            self.bus.select_mode(Mode.PROFILE_POSITION_MODE)
        else:                                                                   # amb exclusió → motor a motor
            for joint in self.config.joints:                                    # iterem els joints
                if joint.motor_id in exclude:                                   # si aquest està exclòs
                    continue                                                    # el saltem
                pkt = encode_mode_selection(                                    # paquet mode_selection
                    joint.motor_id, Mode.PROFILE_POSITION_MODE,
                    fb_cycle1=10, fb_cycle2=10, fb_cycle3=0,
                )
                if self.bus._daemon is not None:                                # bus amb daemon
                    self.bus._daemon.send({"cmd": "raw_bytes", "bytes": list(pkt)})
                else:                                                           # bus amb simulador
                    self.bus._sim.feed_command(pkt)
        self._mode = Mode.PROFILE_POSITION_MODE                                 # recordem el mode del bus

    def reset_all(self) -> None:
        self._require_connected()
        self.bus.reset_all()




    def send_q(self, q_rad: np.ndarray, current_mA: Optional[np.ndarray] = None, velocity_counts: Optional[np.ndarray] = None) -> None:
        self._require_connected()
        if self._mode is None:                                                  # Verifica que s'ha activat un mode primer. Enviar q sense mode seria caos.
            raise RuntimeError("Cal cridar enable_position_control() abans")

        counts = q_to_motor_counts(q_rad, self.config)                          # Converteix radians a counts d'encoder
        if current_mA is None:                                                  # Si no es va passar current_mA, agafa el valor per defecte de cada joint del config.
            current_mA = np.array([j.default_current_mA for j in self.config.joints])
        if velocity_counts is None:                                             # igual per velocitat
            velocity_counts = np.array([j.default_velocity_counts for j in self.config.joints])

        # ----- Camí ràpid: cap motor exclòs → enviament global -----
        exclude = set(self._auto_exclude_motor_ids or ())                       # ids d'exclusió (vuit per defecte)
        if not exclude:                                                         # cas típic (99% dels cops)
            if self._mode == Mode.POSITION_MODE:                                # POSITION_MODE sense rampa (velocitat màxima)
                self.bus.write_positions_pure(current_mA, velocity_counts, counts)
            elif self._mode == Mode.PROFILE_POSITION_MODE:                      # PROFILE_POSITION_MODE amb rampa
                self.bus.write_positions(current_mA, velocity_counts, counts)
            else:                                                               # altres modes no suportats
                raise RuntimeError(f"Mode no suportat per send_q: {self._mode}")
            return                                                              # fet

        # ----- Camí filtrat: hi ha motors exclosos → enviament motor a motor -----
        # Això passa, per exemple, quan el J6 està en CURRENT_MODE (torque) o
        # en RESET (IK5). Si enviéssim encode_position al motor exclòs, el
        # firmware canviaria al moment a POSITION_MODE (perquè el byte de
        # mode va dins el propi paquet) i perdríem el control manual.
        for i, joint in enumerate(self.config.joints):                          # iterem els 6 joints
            if joint.motor_id in exclude:                                       # si aquest està exclòs
                continue                                                        # saltem (no enviem res a aquest motor)
            if self._mode == Mode.POSITION_MODE:                                # paquet POSITION_MODE
                pkt = encode_position(
                    joint.motor_id,
                    int(current_mA[i]),
                    int(velocity_counts[i]),
                    int(counts[i]),
                )
            elif self._mode == Mode.PROFILE_POSITION_MODE:                      # paquet PROFILE_POSITION_MODE
                pkt = encode_profile_position(
                    joint.motor_id,
                    int(current_mA[i]),
                    int(velocity_counts[i]),
                    int(counts[i]),
                    Mode.PROFILE_POSITION_MODE,
                )
            else:                                                               # altres modes no suportats
                raise RuntimeError(f"Mode no suportat per send_q: {self._mode}")
            # Enviem al bus com a raw_bytes (daemon real o simulador)
            if self.bus._daemon is not None:                                    # bus amb daemon
                self.bus._daemon.send({"cmd": "raw_bytes", "bytes": list(pkt)})
            else:                                                               # bus amb simulador
                self.bus._sim.feed_command(pkt)



    def read_q(self) -> np.ndarray:                                             # Llegeix la posició actual dels motors i retorna un array de 6 valors en rad.
        self._require_connected()
        status = self.bus.poll_feedback()                                       # Demana al bus l'últim estat de tots els motors.

        counts = np.zeros(6, dtype=np.int64)                                    # Array de 6 zeros on anirem posant els counts de cada motor.
        for i, j in enumerate(self.config.joints):                              # Itera els 6 joints en ordre
            s = status.get(j.motor_id)                                          # agafa el seu estat
            if s is not None:                                                   # Si existeix, copia la actuator_position (counts després del reductor) a counts[i].
                counts[i] = s.actuator_position

        return motor_counts_to_q(counts, self.config)



    def read_status(self) -> dict:                                              # Retorna l'estat de tots els motors (per debug).
        self._require_connected()
        return self.bus.poll_feedback()

 
    def _require_connected(self):
        if not self._connected:
            raise RuntimeError("Motor controller no està connectat")