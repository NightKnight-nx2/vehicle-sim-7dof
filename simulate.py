"""solve_ivp cozumu + sabit orneklemeli (varsayilan 50 Hz) CSV cikisi."""
from __future__ import annotations

import csv
import numpy as np
from scipy.integrate import solve_ivp

from vehicle_sim.vehicle_model import (
    VehicleParams, dynamics_rhs, initial_state, ackermann_steer_angles,
    IX, IY, IPSI, IVX, IVY,
)
from vehicle_sim.raceline import Raceline
from vehicle_sim.driver import DriverProfile, PurePursuitController


def run_simulation(
    raceline: Raceline,
    profile: DriverProfile,
    vehicle_params: VehicleParams | None = None,
    sample_hz: float = 50.0,
    t_max: float = 60.0,
    v0: float = 8.0,
):
    """
    Tek bir surucu profili icin simulasyonu cozer.

    Donus: dict(t, x, y, yaw, speed, wheel_fl, wheel_fr, wheel_rl, wheel_rr,
                controller, finished_at)
    """
    p = vehicle_params or VehicleParams()
    controller = PurePursuitController(raceline, profile)

    x0, y0, _, _ = raceline.point_at_s(0.0)
    heading0 = raceline.heading[0]
    y_init = initial_state(x0=x0, y0=y0, psi0=heading0, v0=v0, p=p)

    t_eval = np.arange(0.0, t_max, 1.0 / sample_hz)

    def rhs(t, y):
        return dynamics_rhs(t, y, controller, p)

    def finished_event(t, y):
        # solve_ivp'nin erken durmasi icin: kontrolor bitis cizgisine
        # yaklastigini isaretledikten kisa bir sure sonra hiz ~0'a
        # dusunce entegrasyonu durdur (gereksiz uzun kuyruklardan kacinmak
        # icin). Negatiften pozitife gecis olay olarak sayilir.
        speed = np.hypot(y[IVX], y[IVY])
        return (speed - 0.15) if controller.finished else 1.0
    finished_event.terminal = True
    finished_event.direction = -1

    # Not: rtol/atol cok siki (ör. 1e-8) secilirse, lastik/teker
    # dinamiginin tire modelindeki friction-circle np.where kaynakli hafif
    # sureksizligi RK45'in adaptif adim buyuklugu kontrolcusunu asiri
    # kucuk adimlara (pratikte "donmus" gibi gorunen bir yavasliga)
    # surukleyebiliyor. rtol=1e-4/atol=1e-3 + max_step=0.01 s, sabit-adimli
    # bir RK4 referansiyla karsilastirmali test edilerek (bkz. proje
    # notlari) yeterince dogru VE kararli bulundu.
    sol = solve_ivp(
        rhs, (0.0, t_max), y_init, method="RK45",
        t_eval=t_eval, max_step=0.01, events=finished_event,
        rtol=1e-4, atol=1e-3,
    )

    t = sol.t
    Y = sol.y
    n = len(t)

    wheel_fl = np.zeros(n)
    wheel_fr = np.zeros(n)
    for i in range(n):
        delta_c, _, _ = controller(t[i], Y[:, i], p)
        d_fl, d_fr, _, _ = ackermann_steer_angles(delta_c, p)
        wheel_fl[i] = np.degrees(d_fl)
        wheel_fr[i] = np.degrees(d_fr)

    result = {
        "t": t,
        "x": Y[IX, :],
        "y": Y[IY, :],
        "yaw_deg": np.degrees(Y[IPSI, :]),
        "speed": np.hypot(Y[IVX, :], Y[IVY, :]),
        "wheel_fl_deg": wheel_fl,
        "wheel_fr_deg": wheel_fr,
        "wheel_rl_deg": np.zeros(n),
        "wheel_rr_deg": np.zeros(n),
        "controller": controller,
        "solved_to": t[-1] if n else 0.0,
        "success": sol.success,
    }
    return result


def write_csv(result: dict, path: str):
    fieldnames = ["time", "x", "y", "yaw_deg", "speed",
                  "wheel_fl_deg", "wheel_fr_deg", "wheel_rl_deg", "wheel_rr_deg"]
    n = len(result["t"])
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(fieldnames)
        for i in range(n):
            w.writerow([
                f"{result['t'][i]:.4f}",
                f"{result['x'][i]:.4f}",
                f"{result['y'][i]:.4f}",
                f"{result['yaw_deg'][i]:.3f}",
                f"{result['speed'][i]:.4f}",
                f"{result['wheel_fl_deg'][i]:.3f}",
                f"{result['wheel_fr_deg'][i]:.3f}",
                f"{result['wheel_rl_deg'][i]:.3f}",
                f"{result['wheel_rr_deg'][i]:.3f}",
            ])
