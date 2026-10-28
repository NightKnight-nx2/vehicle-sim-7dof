"""
Kendi (özgün) Pacejka "Magic Formula" lastik modeli implementasyonumuz.

Bu modül dogrudan hicbir ucuncu parti lastik kutuphanesini import etmez.
Kod, `commonroad-vehicle-models` (GPLv3) paketindeki tire_model.py
fonksiyonlarinin YAPISINI (long./lateral Pacejka katsayilari B,C,D,E ve
combined-slip friction-circle olceklemesi) referans alarak sifirdan
yazilmistir; sayisal katsayilar da kendi (basitlestirilmis, tipik bir
yolcu/spor otomobil lastigine ait) degerlerimizdir. Dogrulama icin
`validate_tire.py` bu modulun ciktilarini commonroad-vehicle-models'in
STMD/MB modellerinde kullanilan tireModel fonksiyonlarinin ciktilariyla
karsilastirir (bkz. o dosyadaki yorumlar).

Klasik "Magic Formula" (Pacejka 1992, sadeleştirilmiş / boy-akış terimsiz):

    F(slip) = D * sin( C * atan( B*slip - E*(B*slip - atan(B*slip)) ) )

    B: stiffness factor
    C: shape factor
    D: peak value (= mu * Fz, yuk-bagimli)
    E: curvature factor

Bu modul hem "saf" (pure) boylamsal/yanal egrileri hem de basit bir
friction-circle (surtunme cemberi) ile birlesik-kayma (combined slip)
olceklemesini saglar.
"""
from __future__ import annotations

import numpy as np
from dataclasses import dataclass


@dataclass
class PacejkaParams:
    """Tek bir lastik icin sadelestirilmis Magic Formula katsayilari.

    Degerler, tipik bir performans lastigi icin makul mertebede secilmis
    "ilk-prensip" (sıfırdan) katsayilardir; belirli bir gercek lastigin
    olcum verisi degildir. mu_nominal, Fz_nominal yukunde D=mu*Fz iliskisini
    kalibre eder; D, yuk arttikca hafif "load sensitivity" (yuk hassasiyeti)
    ile artar ama orantili artmaz (gercek lastiklerde de boyledir).
    """

    # Boylamsal (longitudinal) egri
    B_x: float = 12.0
    C_x: float = 1.65
    E_x: float = -0.6
    mu_x: float = 1.15          # nominal yukte boylamsal tepe surtunme katsayisi

    # Yanal (lateral) egri
    B_y: float = 9.5
    C_y: float = 1.30
    E_y: float = -1.2
    mu_y: float = 1.05          # nominal yukte yanal tepe surtunme katsayisi

    Fz_nominal: float = 4000.0  # N, katsayilarin kalibre edildigi tekerlek yuku
    load_sensitivity: float = 0.20  # yuk arttikca mu'nun ne kadar dustugunu kontrol eder


def _peak_mu(mu_nom: float, Fz: float, Fz_nom: float, sensitivity: float) -> float:
    """Basit dogrusal yuk-hassasiyeti: mu(Fz) = mu_nom * (1 - sensitivity*(Fz/Fz_nom - 1)).

    Gercek lastiklerde D=mu*Fz iliskisi tam dogrusal degildir (yuk arttikca
    mu hafifce duser) - bu, cok basit ama yon olarak doğru bir yaklastirmadir.
    """
    Fz = max(Fz, 1.0)
    ratio = Fz / Fz_nom
    mu = mu_nom * (1.0 - sensitivity * (ratio - 1.0))
    return max(mu, 0.05)


def pure_longitudinal_force(kappa: np.ndarray, Fz: np.ndarray, p: PacejkaParams) -> np.ndarray:
    """Saf boylamsal kayma (kappa, slip ratio, boyutsuz) icin Pacejka kuvveti [N]."""
    mu = _peak_mu(p.mu_x, Fz, p.Fz_nominal, p.load_sensitivity)
    D = mu * Fz
    Bx_kappa = p.B_x * kappa
    return D * np.sin(p.C_x * np.arctan(Bx_kappa - p.E_x * (Bx_kappa - np.arctan(Bx_kappa))))


def pure_lateral_force(alpha: np.ndarray, Fz: np.ndarray, p: PacejkaParams) -> np.ndarray:
    """Saf yanal kayma acisi (alpha, radyan) icin Pacejka kuvveti [N]."""
    mu = _peak_mu(p.mu_y, Fz, p.Fz_nominal, p.load_sensitivity)
    D = mu * Fz
    By_alpha = p.B_y * alpha
    return -D * np.sin(p.C_y * np.arctan(By_alpha - p.E_y * (By_alpha - np.arctan(By_alpha))))


def combined_slip_forces(kappa, alpha, Fz, p: PacejkaParams):
    """
    Birlesik (combined) kayma: once her eksende "saf" Pacejka kuvvetini
    hesapla, sonra ikisini birlikte bir surtunme cemberine (friction
    ellipse) sigacak sekilde olcekle. Bu, tam bir MF combined-slip
    modelinden (ör. commonroad'daki tireModel_combined) daha basittir ama
    ayni fiziksel ilkeye dayanir: Fx ve Fy ayni anda mumkun olan toplam
    surtunmeyi (yaklasik mu*Fz) asamaz.

    Girisler skaler veya ayni sekle sahip numpy dizileri olabilir.
    Donus: (Fx, Fy) [N]
    """
    Fx0 = pure_longitudinal_force(kappa, Fz, p)
    Fy0 = pure_lateral_force(alpha, Fz, p)

    mu_x = _peak_mu(p.mu_x, Fz, p.Fz_nominal, p.load_sensitivity)
    mu_y = _peak_mu(p.mu_y, Fz, p.Fz_nominal, p.load_sensitivity)

    # Friction-ellipse normalize edilmis kuvvet buyuklugu
    ex = Fx0 / np.maximum(mu_x * Fz, 1e-6)
    ey = Fy0 / np.maximum(mu_y * Fz, 1e-6)
    rho = np.sqrt(ex ** 2 + ey ** 2)
    scale = np.where(rho > 1.0, 1.0 / np.maximum(rho, 1e-9), 1.0)

    return Fx0 * scale, Fy0 * scale
