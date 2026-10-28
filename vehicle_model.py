"""
7-DOF (Serbestlik Derecesi) araç dinamiği modeli.

===========================================================================
7 DOF'UN TANIMI (bu proje icin secilen ve dokumante edilen versiyon)
===========================================================================
Görevde belirtilen esneklik kullanılarak şu "en azından" seçenek uygulandı:

    1. vx   - govde-eksenli boylamsal hiz (longitudinal)
    2. vy   - govde-eksenli yanal hiz (lateral)
    3. r    - yaw acisal hizi (yaw rate)
    4. w_fl - on-sol tekerlek donus hizi
    5. w_fr - on-sag tekerlek donus hizi
    6. w_rl - arka-sol tekerlek donus hizi
    7. w_rr - arka-sag tekerlek donus hizi

Bu 7 durum degiskeninin HER BIRININ kendi ataleti / genellestirilmis
kuvveti vardir (m, m, Iz, Iw x4) - yani gercek "dinamik" serbestlik
derecesi tanimina uyarlar (D'Alembert/Lagrange anlaminda: q_i icin bir
M_i*q_i_ddot = Q_i denklemi var).

Ride (dusey sekme), roll (yatis) ve pitch (baş vurma) icin AYRI durum
degiskenleri (ör. z, phi, theta ve bunlarin turevleri - toplam 13 DOF'a
cikan tam bir "7+6" model) EKLENMEDI; bunun yerine bu ucu, tekerlek
normal yuklerini (Fz) etkileyen KUAZI-STATIK bir yuk transferi modeliyle
temsil ediyoruz (bkz. `_wheel_normal_loads`): boylamsal ivme govdeyi
"sanki pitch yapmis gibi" on/arka arasinda; yanal ivme "sanki roll
yapmis gibi" sol/sag arasinda yuk kaydiriyor. Bu, gorev metninde acikca
izin verilen "en azindan longitudinal+lateral+yaw+4 tekerlek" formuluyle
birebir ortusuyor ve dokumante edilmis bir basitlestirmedir.

Ayrica X, Y (dunya-eksenli konum) ve psi (yaw acisi/heading) durumlari
da entegre edilir - bunlar KINEMATIK'tir (kendi ataletleri/kuvvetleri
yoktur, sadece govde hizlarinin dunya eksenine izdusumudur) ve 7
DINAMIK DOF'a DAHIL DEGILDIR; ama araci dunyada konumlandirmak (ve
Unity replay'i icin CSV'ye yazmak) icin gereklidir.

Toplam entegre edilen durum vektoru (10 eleman):
    y = [X, Y, psi, vx, vy, r, w_fl, w_fr, w_rl, w_rr]

===========================================================================
Tekerlek duzeni / isim kurali
===========================================================================
    FL --- FR      (on aks, +x yonunde a kadar ileride)
     |     |
    RL --- RR      (arka aks, -x yonunde b kadar geride)

Govde eksen takimi: x ileri, y sola, z yukari (sag-el kurali), psi=0 iken
arac +x yonune bakiyor. Bu, Unity'nin sol-el (x-sag, y-yukari, z-ileri)
eksen takimindan FARKLIDIR - donusum CSV yazicisinda / Unity tarafinda
acikca yapilir (bkz. simulate.py ve ReplayPlayer.cs).
"""
from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from vehicle_sim.tire_model import PacejkaParams, combined_slip_forces


@dataclass
class VehicleParams:
    mass: float = 1400.0          # kg
    Iz: float = 2100.0            # kg*m^2, yaw eylemsizlik momenti
    a: float = 1.25               # m, CG'den on aksa mesafe
    b: float = 1.45               # m, CG'den arka aksa mesafe
    track_width: float = 1.55     # m
    cg_height: float = 0.50       # m, kuazi-statik yuk transferi icin
    wheel_radius: float = 0.31    # m
    wheel_inertia: float = 1.6    # kg*m^2, tek tekerlek (jant+lastik) ataleti

    drag_coeff: float = 0.32
    frontal_area: float = 2.2     # m^2
    air_density: float = 1.225    # kg/m^3
    rolling_resistance: float = 0.014  # boyutsuz Crr

    max_engine_torque: float = 320.0   # N*m, tek arka teker basina, dusuk hizda
    max_brake_torque: float = 2200.0   # N*m, tek teker basina
    brake_bias_front: float = 0.6      # firen torkunun on aksa giden orani

    tire_front: PacejkaParams = field(default_factory=PacejkaParams)
    tire_rear: PacejkaParams = field(default_factory=PacejkaParams)

    g: float = 9.81


# Durum vektoru indeksleri (okunabilirlik icin)
IX, IY, IPSI, IVX, IVY, IR, IWFL, IWFR, IWRL, IWRR = range(10)

WHEEL_NAMES = ("FL", "FR", "RL", "RR")


def wheel_positions(p: VehicleParams):
    """Govde eksenindeki (x,y) tekerlek konumlari: FL, FR, RL, RR sirasiyla."""
    return np.array([
        [p.a, p.track_width / 2.0],
        [p.a, -p.track_width / 2.0],
        [-p.b, p.track_width / 2.0],
        [-p.b, -p.track_width / 2.0],
    ])


def ackermann_steer_angles(delta_c: float, p: VehicleParams):
    """
    Tek-iz (bicycle model) esdeger direksiyon acisi delta_c'den, on
    sol/sag tekerlekler icin Ackermann-duzeltilmis acilari hesaplar.
    Arka tekerlekler direksiyon almaz (0).

    delta_c ~ 0 oldugunda (duz gidis) sayisal kararlilik icin lineer
    yaklasima duser.
    """
    L = p.a + p.b
    tw = p.track_width
    if abs(delta_c) < 1e-4:
        return 0.0, 0.0, 0.0, 0.0

    # Donus yaricapindan (L/tan(delta_c)) ic/dis tekerlek acilarini bul.
    R = L / np.tan(delta_c)
    delta_inner = np.arctan(L / (abs(R) - tw / 2.0)) if abs(R) > tw / 2.0 else np.pi / 2.0 * np.sign(delta_c)
    delta_outer = np.arctan(L / (abs(R) + tw / 2.0))

    if delta_c > 0:  # sola donus: sol teker ic (daha keskin acili)
        delta_fl = np.sign(delta_c) * delta_inner
        delta_fr = np.sign(delta_c) * delta_outer
    else:            # saga donus: sag teker ic
        delta_fr = np.sign(delta_c) * delta_inner
        delta_fl = np.sign(delta_c) * delta_outer

    return delta_fl, delta_fr, 0.0, 0.0


def _wheel_normal_loads(p: VehicleParams, ax: float, ay: float):
    """
    Kuazi-statik dikey yuk dagilimi: statik dagilim + boylamsal (pitch
    yerine gecen) + yanal (roll yerine gecen) yuk transferi.
    Donus sirasi: FL, FR, RL, RR.
    """
    m, g, h, tw = p.mass, p.g, p.cg_height, p.track_width
    L = p.a + p.b

    Fz_front_static = m * g * p.b / L
    Fz_rear_static = m * g * p.a / L

    dFz_long = m * ax * h / L  # ax>0 (ivmelenme) -> arkaya yuk kaydirir
    Fz_front_axle = max(Fz_front_static - dFz_long, 1.0)
    Fz_rear_axle = max(Fz_rear_static + dFz_long, 1.0)

    # Toplam yanal transferi aks agirlik oranina gore paylastir (roll
    # stiffness dagilimi acikca modellenmedigi icin basit/yaklasik).
    dFz_lat_total = m * ay * h / tw
    dFz_lat_front = dFz_lat_total * (Fz_front_static / (m * g))
    dFz_lat_rear = dFz_lat_total * (Fz_rear_static / (m * g))

    Fz_fl = max(Fz_front_axle / 2.0 - dFz_lat_front, 20.0)
    Fz_fr = max(Fz_front_axle / 2.0 + dFz_lat_front, 20.0)
    Fz_rl = max(Fz_rear_axle / 2.0 - dFz_lat_rear, 20.0)
    Fz_rr = max(Fz_rear_axle / 2.0 + dFz_lat_rear, 20.0)

    return np.array([Fz_fl, Fz_fr, Fz_rl, Fz_rr])


def wheel_slip(vx, vy, r, omega, steer, pos, p: VehicleParams, v_eps=0.5):
    """Tek bir tekerlek icin slip ratio (kappa) ve slip angle (alpha, rad)."""
    x_i, y_i = pos
    vx_w = vx - r * y_i
    vy_w = vy + r * x_i

    c, s = np.cos(steer), np.sin(steer)
    vx_t = vx_w * c + vy_w * s     # tekerlek dogrultusunda hiz
    vy_t = -vx_w * s + vy_w * c    # tekerlege dik hiz

    # Slip ratio: paydayi kucuk hizlarda sifira bolunmeyi onlemek icin
    # v_eps ile sinirla (durgun/cok yavas haldeki tekil davranisi engeller).
    kappa = (omega * p.wheel_radius - vx_t) / max(abs(vx_t), v_eps)
    # Slip angle: atan2 kullanip vx_t'nin isaretini koruyoruz ki geri
    # giderken de (vx_t<0) fizigin dogru tarafinda kalalim.
    alpha = np.arctan2(vy_t, vx_t if abs(vx_t) > v_eps else np.sign(vx_t + 1e-9) * v_eps)
    return kappa, alpha, (vx_t, vy_t)


def dynamics_rhs(t, y, controls_fn, p: VehicleParams):
    """
    scipy.integrate.solve_ivp icin sag taraf (RHS) fonksiyonu.

    controls_fn(t, y, p) -> (delta_c, throttle, brake)
        delta_c : tek-iz esdeger direksiyon acisi [rad]
        throttle: [0, 1]
        brake   : [0, 1]
    """
    X, Y, psi, vx, vy, r = y[IX], y[IY], y[IPSI], y[IVX], y[IVY], y[IR]
    omegas = y[IWFL:IWRR + 1]

    delta_c, throttle, brake = controls_fn(t, y, p)
    d_fl, d_fr, d_rl, d_rr = ackermann_steer_angles(delta_c, p)
    steers = (d_fl, d_fr, d_rl, d_rr)

    positions = wheel_positions(p)

    # Onceki adimin ivmesini kabaca tahmin etmek yerine, yuk transferini
    # bu adimin kuvvet toplamindan hesaplamak dairesel bir bagimlilik
    # yaratir; pratikte kucuk bir sabit-nokta yaklasimi (bir onceki r/vx/vy
    # turevini kullanmadan, once sifir ivmeyle Fz hesapla, kuvvetleri bul,
    # ivmeyi guncelle, Fz'yi tekrar hesapla) yeterince kararlidir.
    ax_est, ay_est = 0.0, 0.0
    for _ in range(2):
        Fz = _wheel_normal_loads(p, ax_est, ay_est)

        Fx_body = np.zeros(4)
        Fy_body = np.zeros(4)
        for i in range(4):
            kappa, alpha, _ = wheel_slip(vx, vy, r, omegas[i], steers[i], positions[i], p)
            tire = p.tire_front if i < 2 else p.tire_rear
            Fx_t, Fy_t = combined_slip_forces(kappa, alpha, Fz[i], tire)
            c, s = np.cos(steers[i]), np.sin(steers[i])
            Fx_body[i] = Fx_t * c - Fy_t * s
            Fy_body[i] = Fx_t * s + Fy_t * c

        F_drag = 0.5 * p.air_density * p.drag_coeff * p.frontal_area * vx * abs(vx)
        Sum_Fx = np.sum(Fx_body) - F_drag
        Sum_Fy = np.sum(Fy_body)

        ax_est = Sum_Fx / p.mass
        ay_est = Sum_Fy / p.mass

    Sum_Mz = 0.0
    for i in range(4):
        x_i, y_i = positions[i]
        Sum_Mz += x_i * Fy_body[i] - y_i * Fx_body[i]

    vx_dot = Sum_Fx / p.mass + vy * r
    vy_dot = Sum_Fy / p.mass - vx * r
    r_dot = Sum_Mz / p.Iz

    X_dot = vx * np.cos(psi) - vy * np.sin(psi)
    Y_dot = vx * np.sin(psi) + vy * np.cos(psi)
    psi_dot = r

    # Teker donus dinamigi
    omega_dot = np.zeros(4)
    for i in range(4):
        kappa, alpha, _ = wheel_slip(vx, vy, r, omegas[i], steers[i], positions[i], p)
        tire = p.tire_front if i < 2 else p.tire_rear
        Fx_t, _ = combined_slip_forces(kappa, alpha, Fz[i], tire)

        is_rear = i >= 2
        drive_torque = throttle * p.max_engine_torque if is_rear else 0.0  # RWD
        bias = p.brake_bias_front if i < 2 else (1.0 - p.brake_bias_front)
        brake_torque = brake * p.max_brake_torque * bias * np.sign(omegas[i]) if abs(omegas[i]) > 0.05 else brake * p.max_brake_torque * bias
        roll_resist = p.rolling_resistance * Fz[i] * p.wheel_radius * np.sign(omegas[i])

        omega_dot[i] = (drive_torque - brake_torque - roll_resist - Fx_t * p.wheel_radius) / p.wheel_inertia

    dydt = np.zeros(10)
    dydt[IX] = X_dot
    dydt[IY] = Y_dot
    dydt[IPSI] = psi_dot
    dydt[IVX] = vx_dot
    dydt[IVY] = vy_dot
    dydt[IR] = r_dot
    dydt[IWFL:IWRR + 1] = omega_dot
    return dydt


def initial_state(x0=0.0, y0=0.0, psi0=0.0, v0=5.0, p: VehicleParams = None) -> np.ndarray:
    p = p or VehicleParams()
    omega0 = v0 / p.wheel_radius
    return np.array([x0, y0, psi0, v0, 0.0, 0.0, omega0, omega0, omega0, omega0])
