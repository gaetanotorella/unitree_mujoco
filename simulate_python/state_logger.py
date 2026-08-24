"""Logger della GROUND TRUTH del simulatore MuJoCo (stato vero della base).

Il controller di deploy vede solo il LowState via DDS: la posa VERA della base
(qpos del free joint del pelvis) non è osservabile da lì. Questo logger la
scrive in CSV direttamente dal processo del simulatore, così si può:
  - confrontare la Z vera del bacino con la stima FK del deploy (bias della FK
    a busto inclinato, sag del PD, ritardo del filtro EMA);
  - verificare drift XY, orientamento vero vs IMU, velocità della base.

[V6.2] Aggiunto il logging dei CONTATTI dei piedi (foot_contacts.csv): per ogni
piede e per ogni step diagnostica il contatto piede-pavimento — numero di punti,
centro di pressione (CoP) nel frame del piede (per capire se il carico e' sul
BORDO o distribuito), forza normale, e margine di slittamento (rapporto forza
tangenziale / attrito disponibile: ~1 = piede che slitta attivamente). Serve a
capire perche' il robot deriva in MuJoCo ma non in IsaacSim (Esperimento A).

Allineamento temporale: la colonna `time` è mj_data.time, lo STESSO clock che
il bridge pubblica in LowState.tick (in ms). Nei CSV del deploy la colonna
`tick`/1000 combacia quindi con `time` di questo file.

Scrittura in streaming con flush periodico (crash-safe come il logger deploy).
"""

from __future__ import annotations

import csv
import os
import time

import numpy as np
import mujoco


class GroundTruthLogger:
    """Logga la posa base VERA + diagnostica dei contatti dei piedi su due CSV."""

    HEADER = [
        "time",
        "base_x", "base_y", "base_z",
        "quat_w", "quat_x", "quat_y", "quat_z",
        "vx", "vy", "vz",
        "wx", "wy", "wz",
    ]

    # una riga per PIEDE per step campionato
    CONTACT_HEADER = [
        "time", "foot",       # foot = 'L' | 'R'
        "ncon",               # n punti di contatto attivi su questo piede
        "fn_total",           # forza normale totale (N)
        "cop_x", "cop_y",     # centro di pressione nel frame del piede (m): x=avanti, y=laterale
        "ft_total",           # forza tangenziale totale (N)
        "slip_ratio",         # ft / (mu * fn): ~1 = al limite d'attrito (slitta)
        "mz_torsional",       # coppia torsionale al contatto (Nm), se condim>=4
    ]

    def __init__(self, mj_model, out_dir: str, log_dt: float = 0.01,
                 flush_every_s: float = 0.5, contact_log_dt: float = 0.02):
        self.m = mj_model
        os.makedirs(out_dir, exist_ok=True)
        run_dir = os.path.join(out_dir, time.strftime("sim_%Y%m%d_%H%M%S"))
        os.makedirs(run_dir, exist_ok=True)
        self.path = os.path.join(run_dir, "ground_truth.csv")
        self.contact_path = os.path.join(run_dir, "foot_contacts.csv")
        self.log_dt = float(log_dt)
        self.contact_log_dt = float(contact_log_dt)
        self.flush_every_s = float(flush_every_s)

        # --- risoluzione dei corpi/geom dei piedi (una volta) ---
        self._foot_body = {}
        for side, name in (("L", "left_ankle_roll_link"), ("R", "right_ankle_roll_link")):
            try:
                self._foot_body[side] = mujoco.mj_name2id(
                    mj_model, mujoco.mjtObj.mjOBJ_BODY, name)
            except Exception:
                self._foot_body[side] = -1
        # geom_bodyid per mappare un contatto -> corpo
        self._geom_bodyid = mj_model.geom_bodyid.copy()

        self._f = open(self.path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(self.HEADER)

        self._fc = open(self.contact_path, "w", newline="")
        self._wc = csv.writer(self._fc)
        self._wc.writerow(self.CONTACT_HEADER)

        self._next_t = 0.0
        self._next_ct = 0.0
        self._last_flush = time.monotonic()
        self._force6 = np.zeros(6, dtype=np.float64)
        print(f"[state_logger] Ground truth in: {self.path}")
        print(f"[state_logger] Contatti piedi in: {self.contact_path}")

    # ------------------------------------------------------------------
    def log(self, mj_data) -> None:
        """Da chiamare dopo mj_step (dentro il lock)."""
        t = float(mj_data.time)
        if t >= self._next_t:
            self._next_t += self.log_dt
            if t >= self._next_t:
                self._next_t = t + self.log_dt
            self._log_base(mj_data, t)
        if t >= self._next_ct:
            self._next_ct += self.contact_log_dt
            if t >= self._next_ct:
                self._next_ct = t + self.contact_log_dt
            self._log_contacts(mj_data, t)

        now = time.monotonic()
        if now - self._last_flush >= self.flush_every_s:
            self._f.flush()
            self._fc.flush()
            self._last_flush = now

    def _log_base(self, mj_data, t: float) -> None:
        qpos = mj_data.qpos   # free joint: [x y z qw qx qy qz | giunti...]
        qvel = mj_data.qvel   # [vx vy vz wx wy wz | giunti...]
        row = [format(t, ".4f")]
        row += [format(float(qpos[i]), ".6g") for i in range(7)]
        row += [format(float(qvel[i]), ".6g") for i in range(6)]
        self._w.writerow(row)

    def _log_contacts(self, mj_data, t: float) -> None:
        # accumulo per piede: liste di (pos_world[3], f_contact_frame[6], frame[9], mu)
        per_foot = {"L": [], "R": []}
        for i in range(mj_data.ncon):
            con = mj_data.contact[i]
            b1 = self._geom_bodyid[con.geom1]
            b2 = self._geom_bodyid[con.geom2]
            for side, bid in self._foot_body.items():
                if bid >= 0 and (b1 == bid or b2 == bid):
                    mujoco.mj_contactForce(self.m, mj_data, i, self._force6)
                    per_foot[side].append(
                        (np.array(con.pos), self._force6.copy(),
                         np.array(con.frame).reshape(3, 3), float(con.friction[0])))
                    break

        for side in ("L", "R"):
            cons = per_foot[side]
            bid = self._foot_body[side]
            if not cons or bid < 0:
                self._wc.writerow([format(t, ".4f"), side, 0, "0", "0", "0", "0", "0", "0"])
                continue
            # forza normale totale e CoP (media pesata sulla forza normale)
            fn = np.array([c[1][0] for c in cons])              # componente normale (frame axis 0)
            fn_tot = float(fn.sum())
            pos = np.array([c[0] for c in cons])                 # [n,3] world
            if fn_tot > 1e-6:
                cop_w = (pos * fn[:, None]).sum(axis=0) / fn_tot
            else:
                cop_w = pos.mean(axis=0)
            # CoP nel frame del PIEDE (x=avanti, y=laterale): R_foot^T (cop - origin_foot)
            foot_pos = mj_data.xpos[bid]
            foot_mat = mj_data.xmat[bid].reshape(3, 3)
            cop_foot = foot_mat.T @ (cop_w - foot_pos)
            # forza tangenziale: somma vettoriale delle componenti tangenti (frame axes 1,2)
            # ricostruita in world per sommare correttamente tra punti
            ft_vec = np.zeros(3)
            mz = 0.0
            for _pos, f6, frame, _mu in cons:
                # frame righe = assi contatto [normale, t1, t2]; forza in quel frame
                ft_vec += f6[1] * frame[1] + f6[2] * frame[2]
                if f6.shape[0] >= 6:
                    mz += f6[3]                                   # coppia torsionale (condim>=4)
            ft_tot = float(np.linalg.norm(ft_vec))
            mu = cons[0][3]
            slip = ft_tot / (mu * fn_tot) if fn_tot > 1e-6 else 0.0
            self._wc.writerow([
                format(t, ".4f"), side, len(cons),
                format(fn_tot, ".4g"),
                format(float(cop_foot[0]), ".4g"), format(float(cop_foot[1]), ".4g"),
                format(ft_tot, ".4g"), format(slip, ".3f"), format(mz, ".4g"),
            ])

    def close(self) -> None:
        for f in (self._f, self._fc):
            try:
                f.flush()
                f.close()
            except Exception:
                pass
        print(f"[state_logger] Chiuso: {self.path}")
