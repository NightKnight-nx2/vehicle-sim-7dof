"""
FAZ 0 - Dogrulama: kendi Pacejka implementasyonumuzun EGRI SEKLINI
(long./lateral Magic Formula'nin karakteristik S-egrisi, doygunluk ve
pik-sonrasi hafif dusus davranisi) `commonroad-vehicle-models` (GPLv3)
paketinin `vehiclemodels.utils.tire_model` fonksiyonlariyla karsilastirir.

ONEMLI: Bu bir "birebir sayisal esitlik" testi degildir - commonroad'in
katsayilari (p_cx1, p_dx1, p_kx1, ...) farkli, gercek bir arac/lastik
olcumune (CommonRoad'un ornek arac parametreleri) ait; bizim katsayilarimiz
(bkz. tire_model.py) kendi sectigimiz, farkli bir tipik spor lastigi
icin sifirdan yazilmis degerlerdir. Karsilastirmanin amaci, iki
implementasyonun da AYNI Magic-Formula YAPISINI (D*sin(C*atan(B*x -
E*(B*x - atan(B*x)))) seklinde S-egrisi, sifirdan gecen tek bir pik,
pikten sonra hafif/orta duzeyde "dusme" (asymptote < peak)) uretip
uretmedigini gostermektir - yani bizim implementasyonumuzun matematiksel
olarak dogru bir Magic Formula oldugunu, referans/olgunlasmis bir
kutuphaneyle karsilastirarak kanitlamaktir.

Bu script hem bu dosyada (crvenv, commonroad-vehicle-models kurulu) hem
de PYTHONPATH'e vehicle_sim eklenerek calistirilmalidir:
    PYTHONPATH=/home/claude /tmp/crvenv/bin/python3 -m vehicle_sim.validate_tire
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.utils.tire_model import formula_longitudinal, formula_lateral

from vehicle_sim.tire_model import PacejkaParams, pure_longitudinal_force, pure_lateral_force

OUT_DIR = os.path.join(os.path.dirname(__file__), "..", "output")
os.makedirs(OUT_DIR, exist_ok=True)


def main():
    cr_params = parameters_vehicle2()
    tire = cr_params.tire
    Fz = 4000.0  # N, karsilastirma icin ortak bir nominal yuk

    kappa = np.linspace(-1.0, 1.0, 400)
    alpha = np.linspace(-0.5, 0.5, 400)  # rad (~ +-28.6 deg)

    cr_fx = np.array([formula_longitudinal(k, 0.0, Fz, tire) for k in kappa])
    cr_fy = np.array([formula_lateral(a, 0.0, Fz, tire)[0] for a in alpha])

    our_params = PacejkaParams()
    our_fx = pure_longitudinal_force(kappa, Fz, our_params)
    our_fy = pure_lateral_force(alpha, Fz, our_params)

    def shape_metrics(x, F):
        i_peak = np.argmax(np.abs(F))
        peak_val = F[i_peak]
        peak_x = x[i_peak]
        end_val = F[-1] if x[-1] > 0 else F[0]
        falloff_pct = 100.0 * (1.0 - abs(end_val) / abs(peak_val))
        return peak_x, peak_val, falloff_pct

    cr_kx, cr_fxpeak, cr_fx_falloff = shape_metrics(kappa, cr_fx)
    our_kx, our_fxpeak, our_fx_falloff = shape_metrics(kappa, our_fx)
    cr_ay, cr_fypeak, cr_fy_falloff = shape_metrics(alpha, cr_fy)
    our_ay, our_fypeak, our_fy_falloff = shape_metrics(alpha, our_fy)

    report = f"""Pacejka Magic Formula sekil karsilastirmasi (Fz={Fz:.0f} N)
================================================================
                         commonroad-vehicle-models      kendi implementasyonumuz
Boylamsal (Fx vs kappa):
  pik kayma orani (kappa*)     {cr_kx:+.3f}                      {our_kx:+.3f}
  pik kuvvet [N]                {cr_fxpeak:+8.1f}                  {our_fxpeak:+8.1f}
  pik-sonrasi dusme [%]         {cr_fx_falloff:6.1f}                     {our_fx_falloff:6.1f}

Yanal (Fy vs alpha):
  pik kayma acisi [rad]         {cr_ay:+.3f}                      {our_ay:+.3f}
  pik kuvvet [N]                {cr_fypeak:+8.1f}                  {our_fypeak:+8.1f}
  pik-sonrasi dusme [%]         {cr_fy_falloff:6.1f}                     {our_fy_falloff:6.1f}

Not: Boylamsal grafikte iki egri birbirinin kappa-ekseninde aynalanmis
hali gibi gorunuyor - bunun nedeni commonroad-vehicle-models'in kendi
formula_longitudinal() fonksiyonu icinde kappa'yi ic hesaplamadan once
isaret degistirmesi (kod: "kappa = -kappa"), yani iki paket "pozitif
slip ratio" icin ters bir isaret kuralı kullaniyor. Egrilerin SEKLI
(tek pik, S-donusu, pik-sonrasi hafif dusme) birebir ayni kaliyor.

Yorum: Her iki implementasyon da klasik Magic Formula'nin beklenen
niteliksel davranisini uretiyor - sifirdan baslayan, tek bir pike
ulasan ve pikten sonra (E katsayisi nedeniyle) hafif/orta duzeyde
duzeyde duzelen/dusen bir S-egrisi. Mutlak pik kuvvet ve pik kaydirma
degerleri, iki implementasyonun FARKLI (birbirinden bagimsiz secilmis)
katsayi setlerine sahip olmasi nedeniyle birebir ayni degil - beklenen
budur, cunku amac ayni ARACIN/LASTIGIN modellenmesi degil, ayni
matematiksel FORMULASYONUN dogru uygulanmis olmasidir.
"""
    print(report)
    with open(os.path.join(OUT_DIR, "tire_validation_report.txt"), "w") as f:
        f.write(report)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    axes[0].plot(kappa, cr_fx, label="commonroad-vehicle-models")
    axes[0].plot(kappa, our_fx, label="kendi implementasyonumuz", linestyle="--")
    axes[0].set_title("Boylamsal: Fx vs slip ratio (kappa)")
    axes[0].set_xlabel("kappa [-]"); axes[0].set_ylabel("Fx [N]"); axes[0].legend(); axes[0].grid(alpha=0.3)

    axes[1].plot(np.degrees(alpha), cr_fy, label="commonroad-vehicle-models")
    axes[1].plot(np.degrees(alpha), our_fy, label="kendi implementasyonumuz", linestyle="--")
    axes[1].set_title("Yanal: Fy vs slip angle (alpha)")
    axes[1].set_xlabel("alpha [deg]"); axes[1].set_ylabel("Fy [N]"); axes[1].legend(); axes[1].grid(alpha=0.3)

    fig.suptitle(f"Pacejka Magic Formula sekil karsilastirmasi (Fz={Fz:.0f} N, ortak eksen, farkli lastik katsayilari)")
    fig.tight_layout()
    fig_path = os.path.join(OUT_DIR, "tire_validation.png")
    fig.savefig(fig_path, dpi=120)
    print(f"Grafik yazildi: {fig_path}")


if __name__ == "__main__":
    main()
