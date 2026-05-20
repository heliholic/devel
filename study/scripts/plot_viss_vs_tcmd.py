#!/usr/bin/env python3
"""
Plot steady-state induced velocity v_i,ss as a function of commanded thrust T_cmd
for several axial flow speeds V_inf.

Solves the momentum closure (tail_rotor_dynamics.tex, eq. 71)

    T_cmd = g_m * v_i,ss * sqrt((V_inf + v_i,ss)^2 + V_perp^2),     g_m = 2*rho*A_t

for v_i,ss on the positive-thrust working branch.
"""

from __future__ import annotations

import argparse

import matplotlib.pyplot as plt
import numpy as np

from thrust_solver import (
    AxialQuadraticSolver,
    BisectionSolver,
    MomentumClosure,
    RotorParams,
)


def viss_curve(
    T_cmd: np.ndarray,
    V_inf: float,
    V_perp: float,
    params: RotorParams,
) -> np.ndarray:
    """v_i,ss(T_cmd) on the positive working branch."""
    if V_perp == 0.0:
        solver = AxialQuadraticSolver()
    else:
        solver = BisectionSolver(max_iter=80, tol=1.0e-10)

    out = np.empty_like(T_cmd, dtype=float)
    for i, T in enumerate(T_cmd):
        problem = MomentumClosure(
            T_cmd=float(T), V_inf=V_inf, V_perp=V_perp, params=params
        )
        out[i] = solver.solve(problem)
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--T-max", type=float, default=30.0, help="Max T_cmd [N]")
    parser.add_argument("--n", type=int, default=400, help="Samples in T_cmd sweep")
    parser.add_argument(
        "--V-inf",
        type=float,
        nargs="+",
        default=[-5.0, -2.0, 0.0, 2.0, 5.0, 10.0, 20.0],
        help="Axial flow speeds to plot [m/s]",
    )
    parser.add_argument("--V-perp", type=float, default=0.0, help="Edgewise flow [m/s]")
    parser.add_argument("--rho", type=float, default=1.225)
    parser.add_argument("--A-t", type=float, default=0.0616, help="Disc area [m^2]")
    parser.add_argument("--save", type=str, default=None, help="Output image path")
    parser.add_argument("--dpi", type=int, default=200, help="DPI for saved image")
    args = parser.parse_args()

    params = RotorParams(rho=args.rho, A_t=args.A_t)
    T_cmd = np.linspace(0.0, args.T_max, args.n)

    fig, ax = plt.subplots(figsize=(8.0, 5.5), dpi=args.dpi)
    cmap = plt.get_cmap("viridis")
    V_list = sorted(args.V_inf)
    norm = plt.Normalize(vmin=min(V_list), vmax=max(V_list))

    for V_inf in V_list:
        v_iss = viss_curve(T_cmd, V_inf, args.V_perp, params)
        ax.plot(
            T_cmd,
            v_iss,
            color=cmap(norm(V_inf)),
            label=f"V_inf = {V_inf:+.1f} m/s",
            lw=1.6,
        )

    g_m = params.g_m
    v_hover = np.sqrt(np.maximum(T_cmd, 0.0) / g_m)
    ax.plot(T_cmd, v_hover, "k--", lw=1.0, alpha=0.6, label="hover: sqrt(T/g_m)")

    ax.set_xlabel("Commanded thrust  T_cmd  [N]")
    ax.set_ylabel("Steady induced velocity  v_i,ss  [m/s]")
    ax.set_title(
        f"v_i,ss vs T_cmd  (rho={params.rho}, A_t={params.A_t} m^2, "
        f"V_perp={args.V_perp} m/s)"
    )
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9)
    ax.set_xlim(0.0, args.T_max)
    ax.set_ylim(bottom=0.0)
    fig.tight_layout()

    if args.save:
        fig.savefig(args.save, dpi=args.dpi)
        print(f"saved {args.save} (dpi={args.dpi})")
    else:
        plt.show()


if __name__ == "__main__":
    main()
