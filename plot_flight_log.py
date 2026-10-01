import os
import argparse
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

from drone_control import WAYPOINTS, WP_COLORS

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", default=os.path.join(HERE, "flight_log.csv"))
    parser.add_argument("--out", default=os.path.join(HERE, "..", "docs", "img", "trayectoria_drones.png"))
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()
    if not args.show:
        matplotlib.use("Agg")

    with open(args.log) as f:
        header = f.readline().strip().split(",")
    data = np.genfromtxt(args.log, delimiter=",", skip_header=1, dtype=None, encoding=None)
    t = np.array([row[0] for row in data], dtype=float)
    center = np.array([[row[2], row[3], row[4]] for row in data], dtype=float)
    n_drones = (len(header) - 5) // 3
    drones = np.array([[list(row)[5 + 3 * i: 8 + 3 * i] for i in range(n_drones)] for row in data], dtype=float)

    fig = plt.figure(figsize=(13, 5.5))
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    for i in range(n_drones):
        ax.plot(drones[:, i, 0], drones[:, i, 1], drones[:, i, 2], lw=0.8, alpha=0.7)
    ax.plot(center[:, 0], center[:, 1], center[:, 2], "k--", lw=1.5, label="centro de formación")
    for name, wp in WAYPOINTS.items():
        ax.scatter(*wp, s=120, color=WP_COLORS[name], edgecolor="k", zorder=5)
        ax.text(wp[0], wp[1], wp[2] + 0.12, name, fontsize=14, weight="bold", color=WP_COLORS[name])
    ax.set_xlabel("x [m]"), ax.set_ylabel("y [m]"), ax.set_zlabel("z [m]")
    ax.set_title(f"Trayectoria de {n_drones} drones: A → B → C")
    ax.legend(loc="upper left")

    ax2 = fig.add_subplot(1, 2, 2)
    mean = drones.mean(axis=1)
    for k, lab in enumerate("xyz"):
        ax2.plot(t, mean[:, k], label=f"{lab} real (promedio)")
        ax2.plot(t, center[:, k], "--", color=ax2.lines[-1].get_color(), alpha=0.6, label=f"{lab} comandado")
    ax2.set_xlabel("tiempo [s]"), ax2.set_ylabel("posición [m]")
    ax2.set_title("Posición real vs. comandada")
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8, ncol=2)

    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    fig.savefig(args.out, dpi=130)
    print("Gráfica guardada en", os.path.abspath(args.out))
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
