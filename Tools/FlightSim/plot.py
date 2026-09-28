"""Plot flightsim CSV logs into docs/img/*.png.  Usage: python plot.py [csv_dir] [img_dir]"""
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

src = Path(sys.argv[1] if len(sys.argv) > 1 else "out")
dst = Path(sys.argv[2] if len(sys.argv) > 2 else "../../docs/img")
dst.mkdir(parents=True, exist_ok=True)


def load(name):
    with open(src / name, newline="") as f:
        rows = list(csv.DictReader(f))
    return {k: [float(r[k]) for r in rows] for k in rows[0]}


def save(fig, name):
    fig.tight_layout()
    fig.savefig(dst / name, dpi=130)
    plt.close(fig)


d = load("01_takeoff.csv")
fig, ax = plt.subplots(figsize=(7, 3.2))
ax.plot(d["t"], d["z"], label="altitude")
ax.axhline(5, ls="--", c="gray", lw=1, label="setpoint 5 m")
ax.axhspan(4.95, 5.05, color="gray", alpha=0.15, label="±5 cm band")
ax.set(xlabel="time [s]", ylabel="z [m]", title="Takeoff to 5 m (position mode)")
ax.legend(loc="lower right")
save(fig, "takeoff.png")

d = load("02_roll_step.csv")
fig, (a1, a2) = plt.subplots(2, 1, figsize=(7, 5), sharex=True)
a1.plot(d["t"], d["roll"], label="roll")
a1.plot(d["t"], d["roll_sp"], "--", label="setpoint")
a1.set(ylabel="roll [deg]", title="Roll step 0 → 20° (attitude loop)")
a1.legend()
a2.plot(d["t"], d["p"], label="roll rate p")
a2.plot(d["t"], d["p_sp"], "--", label="rate setpoint")
a2.set(xlabel="time [s]", ylabel="rate [deg/s]")
a2.legend()
save(fig, "roll_step.png")

d = load("03_gust.csv")
fig, ax = plt.subplots(figsize=(7, 3.2))
ax.plot(d["t"], d["y"], label="y (gust axis)")
ax.plot(d["t"], d["x"], label="x")
ax.axvspan(1.0, 1.5, color="orange", alpha=0.2, label="6 N push")
ax.set(xlabel="time [s]", ylabel="position [m]", title="Gust rejection during position hold")
ax.legend()
save(fig, "gust.png")

d = load("05_waypoints.csv")
fig, ax = plt.subplots(figsize=(4.8, 4.8))
ax.plot(d["x"], d["y"])
ax.plot([0, 10, 10, 0, 0], [0, 0, 10, 10, 0], "k--", lw=0.8, label="planned")
ax.set(xlabel="x [m]", ylabel="y [m] (left)", title="Waypoint square, 5 m altitude", aspect="equal")
ax.legend()
save(fig, "waypoints.png")

print("wrote", *sorted(p.name for p in dst.glob("*.png")))
