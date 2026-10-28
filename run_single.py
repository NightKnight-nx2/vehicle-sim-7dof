"""FAZ 1: Tek surucu profiliyle (baseline) calistir, CSV + kontrol grafigi uret."""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from vehicle_sim.raceline import build_oval_with_chicane
from vehicle_sim.driver import PROFILES
from vehicle_sim.simulate import run_simulation, write_csv

OUT_DIR = os.path.join(os.path.dirname(__file__), "..", "output")
os.makedirs(OUT_DIR, exist_ok=True)


def main():
    raceline = build_oval_with_chicane()
    profile = PROFILES["baseline"]

    print(f"Raceline uzunlugu: {raceline.length:.1f} m, {len(raceline.s)} nokta")

    result = run_simulation(raceline, profile, t_max=45.0)
    print(f"solve_ivp basarili: {result['success']}, cozulen sure: {result['solved_to']:.2f} s, "
          f"{len(result['t'])} orneklem (50 Hz)")
    print(f"Max hiz: {result['speed'].max()*3.6:.1f} km/h, ortalama hiz: {result['speed'].mean()*3.6:.1f} km/h")
    print(f"Max on-sol direksiyon acisi: {abs(result['wheel_fl_deg']).max():.1f} derece")

    csv_path = os.path.join(OUT_DIR, "baseline.csv")
    write_csv(result, csv_path)
    print(f"CSV yazildi: {csv_path}")

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    axes[0, 0].plot(raceline.x, raceline.y, "k--", lw=1, label="raceline")
    axes[0, 0].plot(result["x"], result["y"], "r-", lw=1.5, label="baseline aracin izi")
    axes[0, 0].set_title("Yol (X-Y)")
    axes[0, 0].set_xlabel("X [m]"); axes[0, 0].set_ylabel("Y [m]")
    axes[0, 0].axis("equal"); axes[0, 0].legend()

    axes[0, 1].plot(result["t"], result["speed"] * 3.6)
    axes[0, 1].set_title("Hiz [km/h]"); axes[0, 1].set_xlabel("t [s]")

    axes[1, 0].plot(result["t"], result["wheel_fl_deg"], label="FL")
    axes[1, 0].plot(result["t"], result["wheel_fr_deg"], label="FR")
    axes[1, 0].set_title("On tekerlek direksiyon acilari [deg]")
    axes[1, 0].set_xlabel("t [s]"); axes[1, 0].legend()

    axes[1, 1].plot(raceline.s, raceline.curvature)
    axes[1, 1].set_title("Raceline egriligi [1/m]")
    axes[1, 1].set_xlabel("s [m]")

    fig.tight_layout()
    fig_path = os.path.join(OUT_DIR, "baseline_validation.png")
    fig.savefig(fig_path, dpi=120)
    print(f"Grafik yazildi: {fig_path}")


if __name__ == "__main__":
    main()
