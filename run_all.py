"""FAZ 2: Uc surucu profilini de (baseline, conservative, aggressive) calistir, 3 CSV + karsilastirma grafigi uret."""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from vehicle_sim.raceline import build_oval_with_chicane
from vehicle_sim.driver import PROFILES
from vehicle_sim.simulate import run_simulation, write_csv

OUT_DIR = os.path.join(os.path.dirname(__file__), "..", "output")
os.makedirs(OUT_DIR, exist_ok=True)

COLORS = {"baseline": "red", "conservative": "blue", "aggressive": "green"}


def export_raceline_csv(raceline, path):
    """Unity tarafinda yolu (LineRenderer) cizebilmek icin merkez cizgiyi de disari yaz."""
    import csv as _csv
    with open(path, "w", newline="") as f:
        w = _csv.writer(f)
        w.writerow(["s", "x", "y"])
        # Her ~1 m'de bir nokta yeterli - Unity'de gereksiz yogun bir
        # LineRenderer/mesh olusturmamak icin inceltiyoruz (ds=0.25 -> ~1m).
        step = max(1, int(round(1.0 / raceline.ds)))
        for i in range(0, len(raceline.s), step):
            w.writerow([f"{raceline.s[i]:.3f}", f"{raceline.x[i]:.3f}", f"{raceline.y[i]:.3f}"])


def main():
    raceline = build_oval_with_chicane()
    export_raceline_csv(raceline, os.path.join(OUT_DIR, "raceline.csv"))
    print(f"Raceline CSV: {os.path.join(OUT_DIR, 'raceline.csv')}")
    results = {}

    for name, profile in PROFILES.items():
        print(f"--- {name} ---")
        result = run_simulation(raceline, profile, t_max=45.0)
        print(f"  solve_ivp success={result['success']}  lap time={result['solved_to']:.2f}s  "
              f"max_speed={result['speed'].max()*3.6:.1f} km/h  avg_speed={result['speed'].mean()*3.6:.1f} km/h")
        csv_path = os.path.join(OUT_DIR, f"{name}.csv")
        write_csv(result, csv_path)
        print(f"  CSV: {csv_path} ({len(result['t'])} satir @ 50 Hz)")
        results[name] = result

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    axes[0].plot(raceline.x, raceline.y, "k--", lw=1, label="raceline")
    for name, result in results.items():
        axes[0].plot(result["x"], result["y"], color=COLORS[name], lw=1.5, label=name)
    axes[0].set_title("Uc surucu profilinin izi"); axes[0].axis("equal"); axes[0].legend()
    axes[0].set_xlabel("X [m]"); axes[0].set_ylabel("Y [m]")

    for name, result in results.items():
        axes[1].plot(result["t"], result["speed"] * 3.6, color=COLORS[name], label=name)
    axes[1].set_title("Hiz profili karsilastirmasi"); axes[1].set_xlabel("t [s]"); axes[1].set_ylabel("km/h")
    axes[1].legend()

    fig.tight_layout()
    fig_path = os.path.join(OUT_DIR, "driver_comparison.png")
    fig.savefig(fig_path, dpi=120)
    print(f"Karsilastirma grafigi: {fig_path}")

    print("\nOzet (lap time = bitis cizgisine ulasma suresi):")
    for name, result in results.items():
        print(f"  {name:13s}: {result['solved_to']:6.2f} s   max {result['speed'].max()*3.6:5.1f} km/h   "
              f"avg {result['speed'].mean()*3.6:5.1f} km/h")


if __name__ == "__main__":
    main()
