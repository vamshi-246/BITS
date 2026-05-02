#!/usr/bin/env python3
"""Plot the paper-style windowed parallel turbo decoder BER sweep."""

import argparse
import csv
import os

import matplotlib.pyplot as plt


def read_curve(path):
    ebn0 = []
    ber = []
    errors = []
    bits = []
    frames = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            ebn0.append(float(row["ebn0_db"]))
            ber.append(float(row["ber"]))
            errors.append(int(row["errors"]))
            bits.append(int(row["bits"]))
            frames.append(int(row["frames"]))
    return ebn0, ber, errors, bits, frames


def main():
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    default_csv = os.path.join(
        repo_root,
        "data",
        "ber_curve_K3200_N2_radix4_R0375.csv",
    )
    default_png = os.path.join(
        repo_root,
        "data",
        "ber_plot_K3200_N2_radix4_R0375.png",
    )

    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", default=default_csv)
    parser.add_argument("--out", default=default_png)
    parser.add_argument(
        "--label",
        default="N=2 radix-4 fixed-point RTL model (I=5.5)",
        help="legend label for the plotted BER curve",
    )
    parser.add_argument(
        "--title",
        default="K=3200, N=2, M=30, radix-4 fixed-point, 5.5 iterations",
        help="plot title",
    )
    args = parser.parse_args()

    ebn0, ber, errors, bits, frames = read_curve(args.csv)
    plot_ber = [y if y > 0.0 else 0.5 / b for y, b in zip(ber, bits)]

    plt.figure(figsize=(9.2, 6.8), dpi=160)
    plt.semilogy(
        ebn0,
        plot_ber,
        color="#2d3139",
        marker="s",
        markersize=7,
        markerfacecolor="none",
        markeredgewidth=2.0,
        linewidth=2.8,
        label=args.label,
    )

    for x, y, py, e, b, fr in zip(ebn0, ber, plot_ber, errors, bits, frames):
        label = f"{e}/{b}" if e else f"<{0.5 / b:.1e}"
        y_text = max(y * 1.35, 1.25e-4)
        y_text = max(py * 1.35, y_text)
        ha = "center"
        if x == min(ebn0):
            ha = "left"
        elif x == max(ebn0):
            ha = "right"
        plt.text(
            x,
            y_text,
            f"{label}\n{fr} fr",
            ha=ha,
            va="bottom",
            fontsize=8,
            color="#2d3139",
        )

    plt.xlabel(r"$E_b/N_0$ [dB]", fontsize=16)
    plt.ylabel("BER", fontsize=16)
    plt.title(args.title, fontsize=14)
    plt.xlim(-0.02, 1.04)
    plt.ylim(1e-5, 1.0)
    plt.xticks([0.0, 0.25, 0.5, 0.75, 1.0])
    plt.grid(True, which="major", linestyle="--", linewidth=0.7, color="#8b9098", alpha=0.8)
    plt.grid(True, which="minor", linestyle=":", linewidth=0.55, color="#8b9098", alpha=0.75)
    plt.legend(loc="upper right", frameon=True, fancybox=False, edgecolor="#2d3139", fontsize=11)
    plt.tight_layout()
    plt.savefig(args.out)
    print(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
