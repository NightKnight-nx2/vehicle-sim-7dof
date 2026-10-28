"""
Surucu profilleri + pure-pursuit tabanli path-following kontroloru.

Uc profil (baseline, conservative, aggressive) ayni raceline'i, ayni
arac modeliyle, sadece asagidaki uc parametre farkliyken takip eder:

    corner_speed_mult : viraj hiz limiti carpani (egriligin izin verdigi
                        max hizin ne kadarini kullanmaya calisir)
    brake_margin      : frenleme mesafesi guvenlik marji (>1 = daha
                        erken/yumusak frenler, sanki araci daha az
                        yavaslatabiliyormus gibi davranarak)
    throttle_aggr     : gaza basma / hizlanma agresifligi carpani
"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from vehicle_sim.raceline import Raceline


@dataclass
class DriverProfile:
    name: str
    corner_speed_mult: float
    brake_margin: float
    throttle_aggr: float
    lookahead_base: float = 8.0      # m, hiz=0 iken pure-pursuit onGorus mesafesi
    lookahead_gain: float = 0.6      # s, hiza bagli ek onGorus (lookahead = base + gain*v)
    max_speed: float = 40.0          # m/s, duz yol hiz tavani
    mu_ref: float = 1.0              # viraj hiz limiti icin varsayilan surtunme katsayisi
    base_max_accel: float = 4.5      # m/s^2, ileri gecis (hizlanma) icin varsayilan tavan
    base_max_decel: float = 7.0      # m/s^2, geri gecis (frenleme) icin varsayilan tavan


PROFILES = {
    "baseline": DriverProfile(
        name="baseline", corner_speed_mult=1.00, brake_margin=1.00, throttle_aggr=1.00,
    ),
    "conservative": DriverProfile(
        name="conservative", corner_speed_mult=0.82, brake_margin=1.35, throttle_aggr=0.70,
        lookahead_base=10.0,
    ),
    "aggressive": DriverProfile(
        name="aggressive", corner_speed_mult=1.15, brake_margin=0.75, throttle_aggr=1.30,
        lookahead_base=6.5,
    ),
}


def compute_speed_profile(raceline: Raceline, profile: DriverProfile) -> np.ndarray:
    """
    Egrilige dayali viraj hiz limiti + ulasilabilirlik (frenleme/hizlanma
    kapasitesi) icin klasik iki gecisli (backward+forward) hiz profili
    optimizasyonu. Sonuc, raceline.s ile ayni uzunlukta bir v_target[] dizisi.
    """
    kappa = np.abs(raceline.curvature)
    v_corner = np.sqrt(profile.mu_ref * 9.81 / np.maximum(kappa, 1e-4)) * profile.corner_speed_mult
    v_limit = np.minimum(v_corner, profile.max_speed)

    n = len(raceline.s)
    v = v_limit.copy()

    # Geriye dogru gecis: her noktadan bir onceki noktaya "frenleyerek"
    # ulasilabilecek max hizi sinirla. brake_margin>1 -> daha erken/az agresif
    # firen kapasitesi varsayarak fren bolgesini efektif olarak genisletir.
    effective_decel = profile.base_max_decel / profile.brake_margin
    for i in range(n - 2, -1, -1):
        ds = raceline.s[i + 1] - raceline.s[i]
        v_max_from_next = np.sqrt(v[i + 1] ** 2 + 2 * effective_decel * ds)
        v[i] = min(v[i], v_max_from_next)

    # Ileri gecis: hizlanma kapasitesiyle sinirla.
    effective_accel = profile.base_max_accel * profile.throttle_aggr
    for i in range(1, n):
        ds = raceline.s[i] - raceline.s[i - 1]
        v_max_from_prev = np.sqrt(v[i - 1] ** 2 + 2 * effective_accel * ds)
        v[i] = min(v[i], v_max_from_prev)

    return v


class PurePursuitController:
    """
    Sabit raceline'i takip eden basit bir pure-pursuit + hiz-takip
    kontroloru. `controls_fn(t, y, p)` imzasiyla vehicle_model.dynamics_rhs
    icine dogrudan verilebilir.
    """

    def __init__(self, raceline: Raceline, profile: DriverProfile):
        self.raceline = raceline
        self.profile = profile
        self.v_target_profile = compute_speed_profile(raceline, profile)
        self._s_guess = 0.0
        self.finished = False

        # Basit PI-benzeri hiz kontroloru icin durum
        self._speed_err_integral = 0.0

    def target_speed_at(self, s: float) -> float:
        return float(np.interp(s, self.raceline.s, self.v_target_profile))

    def __call__(self, t, y, p):
        from vehicle_sim.vehicle_model import IX, IY, IPSI, IVX, IVY

        X, Y, psi, vx, vy = y[IX], y[IY], y[IPSI], y[IVX], y[IVY]
        speed = max(np.hypot(vx, vy), 0.1)

        s_here = self.raceline.nearest_s(X, Y, self._s_guess)
        self._s_guess = s_here
        if s_here >= self.raceline.s[-1] - 0.5:
            self.finished = True

        lookahead = self.profile.lookahead_base + self.profile.lookahead_gain * speed
        s_target = min(s_here + lookahead, self.raceline.s[-1])
        tx, ty, _, _ = self.raceline.point_at_s(s_target)

        # Hedef noktayi govde eksenine tasi
        dx, dy = tx - X, ty - Y
        c, s_ = np.cos(-psi), np.sin(-psi)
        x_body = dx * c - dy * s_
        y_body = dx * s_ + dy * c

        Ld = max(np.hypot(x_body, y_body), 1e-3)
        # Klasik pure-pursuit egrilik komutu: kappa_cmd = 2*y_body / Ld^2
        kappa_cmd = 2.0 * y_body / (Ld ** 2)
        wheelbase = p.a + p.b
        delta_c = np.arctan(kappa_cmd * wheelbase)
        delta_c = np.clip(delta_c, -0.55, 0.55)

        v_target = self.target_speed_at(s_here)
        speed_err = v_target - speed

        if speed_err >= 0:
            throttle = np.clip(speed_err * 0.6, 0.0, 1.0)
            brake = 0.0
        else:
            throttle = 0.0
            brake = np.clip(-speed_err * 0.6, 0.0, 1.0)

        if self.finished:
            throttle = 0.0
            brake = 0.6

        return delta_c, throttle, brake
