#!/usr/bin/env python3
"""
Scalar solvers for steady tail-rotor induced velocity v_i,ss from momentum closure.

    T_cmd = g_m * v_i,ss * sqrt((V_inf + v_i,ss)^2 + V_perp^2)

where g_m = 2 * rho * A_t.

Reference: study/theory/tail_rotor_dynamics.tex (eq:viss_implicit, eq:impl_viss).
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from typing import Protocol

try:
    from scipy.optimize import brentq
except ImportError:
    brentq = None


# ---------------------------------------------------------------------------
# Rotor parameters (700-class worked example, tail_rotor_dynamics.tex S5)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class RotorParams:
    rho: float = 1.225
    A_t: float = 0.0616

    @property
    def g_m(self) -> float:
        return 2.0 * self.rho * self.A_t


# ---------------------------------------------------------------------------
# Momentum closure problem
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class MomentumClosure:
    T_cmd: float
    V_inf: float = 0.0
    V_perp: float = 0.0
    params: RotorParams = RotorParams()

    @property
    def g_m(self) -> float:
        return self.params.g_m

    def thrust(self, v: float) -> float:
        return self.g_m * v * math.hypot(self.V_inf + v, self.V_perp)

    def residual(self, v: float) -> float:
        return self.thrust(v) - self.T_cmd

    def d_residual_dv(self, v: float) -> float:
        s = self.V_inf + v
        p = self.V_perp
        denom = math.hypot(s, p)
        if denom == 0.0:
            return 2.0 * self.g_m * v
        return self.g_m * (denom + (s * s) / denom)


class Solver(Protocol):
    name: str

    def solve(self, problem: MomentumClosure) -> float: ...


# ---------------------------------------------------------------------------
# Solver: hover algebraic (V_inf = V_perp = 0)
# ---------------------------------------------------------------------------

@dataclass
class HoverAlgebraicSolver:
    """v_i,ss = sqrt(T_cmd / g_m). Requires V_inf = V_perp = 0."""

    name: str = "hover_algebraic"

    def solve(self, problem: MomentumClosure) -> float:
        if problem.V_inf != 0.0 or problem.V_perp != 0.0:
            raise ValueError(
                f"{self.name} requires V_inf = V_perp = 0 "
                f"(got V_inf={problem.V_inf}, V_perp={problem.V_perp})"
            )
        if problem.T_cmd < 0.0:
            raise ValueError("T_cmd must be non-negative on positive-thrust branch")
        if problem.T_cmd == 0.0:
            return 0.0
        return math.sqrt(problem.T_cmd / problem.g_m)


# ---------------------------------------------------------------------------
# Solver: axial quadratic (V_perp = 0, V_inf + v_i,ss >= 0)
# ---------------------------------------------------------------------------

@dataclass
class AxialQuadraticSolver:
    """
    v_i,ss = (-V_inf + sqrt(V_inf^2 + 4*T_cmd/g_m)) / 2.

    Requires V_perp = 0 and the normal working branch V_inf + v_i,ss >= 0
    (eq:impl_viss_axial in tail_rotor_dynamics.tex).
    """

    name: str = "axial_quadratic"

    def solve(self, problem: MomentumClosure) -> float:
        if problem.V_perp != 0.0:
            raise ValueError(
                f"{self.name} requires V_perp = 0 (got V_perp={problem.V_perp})"
            )
        if problem.T_cmd < 0.0:
            raise ValueError("T_cmd must be non-negative on positive-thrust branch")
        if problem.T_cmd == 0.0:
            return 0.0

        V_inf = problem.V_inf
        disc = V_inf * V_inf + 4.0 * problem.T_cmd / problem.g_m
        v = 0.5 * (-V_inf + math.sqrt(disc))

        if V_inf + v < 0.0:
            raise ValueError(
                f"{self.name}: branch condition V_inf + v_i,ss >= 0 failed "
                f"(V_inf={V_inf}, v_i,ss={v}); use newton or bisection"
            )
        return v


# ---------------------------------------------------------------------------
# Iterative solvers (general case; bisection suitable for embedded use)
# ---------------------------------------------------------------------------

def hover_guess(problem: MomentumClosure) -> float:
    """Hover induced velocity sqrt(T_cmd / g_m); used as bisection warm start."""
    if problem.T_cmd <= 0.0:
        return 0.0
    return math.sqrt(problem.T_cmd / problem.g_m)


def bracket_upper(problem: MomentumClosure, hi: float) -> float:
    """Expand upper bracket until residual(hi) >= 0 (positive-thrust branch)."""
    if problem.T_cmd <= 0.0:
        return hi
    while problem.residual(hi) < 0.0 and hi < 1.0e4:
        hi *= 2.0
    if problem.residual(0.0) * problem.residual(hi) > 0.0:
        raise RuntimeError("bracket does not straddle root on [0, hi]")
    return hi


def initial_bracket(
    problem: MomentumClosure,
    *,
    v0: float,
    hi_scale: float,
    v_max: float,
) -> tuple[float, float]:
    """
    Bracket [0, hi] with hi tied to warm-start guess v0.

    f(0) < 0 on the positive-thrust branch; hi is increased until f(hi) >= 0.
    Falls back to a full [0, v_max] search if the tight bracket does not straddle.
    """
    lo = 0.0
    hi = max(v0 * hi_scale, 0.5)
    hi = bracket_upper(problem, hi)
    if problem.residual(lo) * problem.residual(hi) > 0.0:
        hi = bracket_upper(problem, v_max)
    return lo, hi


@dataclass
class FirmwareNewtonSolver:
    """
    Embedded-shape solver: axial-quadratic warm start + fixed Newton iterations
    on f(v) = v * sqrt((V_inf+v)^2 + V_perp^2) - T_cmd/g_m.

    Mirrors the C pseudocode in plans/consider-how-to-solve-agile-kettle.md.
    No convergence test, no branching on flight regime — predictable WCET.
    """

    name: str = "fw_newton"
    n_iter: int = 2

    def solve(self, problem: MomentumClosure) -> float:
        if problem.T_cmd <= 0.0:
            return 0.0

        g_m = problem.g_m
        V_inf = problem.V_inf
        V_perp = problem.V_perp
        q = problem.T_cmd / g_m

        v = 0.5 * (-V_inf + math.sqrt(V_inf * V_inf + 4.0 * q))

        for _ in range(self.n_iter):
            u = V_inf + v
            D = math.hypot(u, V_perp)
            f = v * D - q
            fv = (u * (V_inf + 2.0 * v) + V_perp * V_perp) / D
            v -= f / fv

        return v


@dataclass
class BisectionSolver:
    """
    Fixed-interval bisection with optional hover warm start.

    Default bracket: [0, hi_scale * sqrt(T_cmd / g_m)] (expanded if needed).
    Iteration count is bounded by max_iter (predictable WCET).
    """

    name: str = "bisection"
    tol: float = 1e-10
    max_iter: int = 4
    v_max: float = 200.0
    hi_scale: float = 2.0
    use_hover_warm_start: bool = True
    v0: float | None = None

    def solve(self, problem: MomentumClosure) -> float:
        if problem.T_cmd < 0.0:
            raise ValueError("T_cmd must be non-negative on positive-thrust branch")
        if problem.T_cmd == 0.0:
            return 0.0

        if self.use_hover_warm_start:
            guess = self.v0 if self.v0 is not None else hover_guess(problem)
            lo, hi = initial_bracket(
                problem, v0=guess, hi_scale=self.hi_scale, v_max=self.v_max
            )
        else:
            lo = 0.0
            hi = bracket_upper(problem, self.v_max)
        f_lo = problem.residual(lo)

        for _ in range(self.max_iter):
            mid = 0.5 * (lo + hi)
            if hi - lo < self.tol:
                return mid
            f_mid = problem.residual(mid)
            if abs(f_mid) < self.tol:
                return mid
            if f_lo * f_mid <= 0.0:
                hi = mid
            else:
                lo = mid
                f_lo = f_mid

        return 0.5 * (lo + hi)


# ---------------------------------------------------------------------------
# Reference solvers (desktop cross-check)
# ---------------------------------------------------------------------------

@dataclass
class NewtonSolver:
    name: str = "newton"
    tol: float = 1e-10
    max_iter: int = 50
    v0: float = 5.0

    def solve(self, problem: MomentumClosure) -> float:
        v = self.v0
        for _ in range(self.max_iter):
            r = problem.residual(v)
            if abs(r) < self.tol:
                return max(v, 0.0)
            dr = problem.d_residual_dv(v)
            if dr == 0.0:
                break
            v -= r / dr
            v = max(v, 0.0)
        raise RuntimeError(
            f"{self.name} did not converge (residual={problem.residual(v):.3e})"
        )


@dataclass
class BrentqSolver:
    name: str = "brentq"
    v_max: float = 200.0

    def solve(self, problem: MomentumClosure) -> float:
        if brentq is None:
            raise ImportError("scipy is required for BrentqSolver")
        if problem.T_cmd <= 0.0:
            return 0.0
        hi = bracket_upper(problem, self.v_max)
        return brentq(lambda v: problem.residual(v), 0.0, hi)


def default_solvers() -> list[Solver]:
    solvers: list[Solver] = [
        HoverAlgebraicSolver(),
        AxialQuadraticSolver(),
        FirmwareNewtonSolver(),
        NewtonSolver(),
        BisectionSolver(),
    ]
    if brentq is not None:
        solvers.append(BrentqSolver())
    return solvers


def sweep_firmware_newton(
    *,
    T_values: list[float],
    V_inf_values: list[float],
    V_perp_values: list[float],
    n_iter: int = 2,
    params: RotorParams = RotorParams(),
) -> dict:
    """
    Sweep (T, V_inf, V_perp) and compare FirmwareNewtonSolver(n_iter) against
    brentq. Returns worst-case residuals and the operating point that hit them.
    """
    if brentq is None:
        raise ImportError("scipy.brentq is required for the sweep cross-check")

    fw = FirmwareNewtonSolver(n_iter=n_iter)
    ref = BrentqSolver()

    worst_residual = 0.0
    worst_v_err = 0.0
    worst_residual_point = None
    worst_v_err_point = None
    branch_failures: list[tuple[float, float, float]] = []
    n_total = 0

    for T_cmd in T_values:
        if T_cmd <= 0.0:
            continue
        for V_inf in V_inf_values:
            for V_perp in V_perp_values:
                problem = MomentumClosure(
                    T_cmd=T_cmd, V_inf=V_inf, V_perp=V_perp, params=params
                )
                try:
                    v_ref = ref.solve(problem)
                except Exception:
                    continue
                v_fw = fw.solve(problem)
                res = abs(problem.residual(v_fw))
                v_err = abs(v_fw - v_ref)
                n_total += 1

                if V_inf + v_fw < 0.0:
                    branch_failures.append((T_cmd, V_inf, V_perp))

                if res > worst_residual:
                    worst_residual = res
                    worst_residual_point = (T_cmd, V_inf, V_perp, v_fw, v_ref)
                if v_err > worst_v_err:
                    worst_v_err = v_err
                    worst_v_err_point = (T_cmd, V_inf, V_perp, v_fw, v_ref)

    return {
        "n_iter": n_iter,
        "n_total": n_total,
        "worst_residual": worst_residual,
        "worst_residual_point": worst_residual_point,
        "worst_v_err": worst_v_err,
        "worst_v_err_point": worst_v_err_point,
        "branch_failures": branch_failures,
    }


def print_sweep_result(result: dict) -> None:
    print(
        f"\nFirmwareNewtonSolver(n_iter={result['n_iter']}) sweep, "
        f"{result['n_total']} points:"
    )
    wr = result["worst_residual_point"]
    we = result["worst_v_err_point"]
    if wr is not None:
        T, V_inf, V_perp, v_fw, v_ref = wr
        print(
            f"  worst |residual| = {result['worst_residual']:.3e} N "
            f"at T={T:.2f} N, V_inf={V_inf:+.2f} m/s, V_perp={V_perp:.2f} m/s "
            f"(v_fw={v_fw:.6f}, v_ref={v_ref:.6f})"
        )
    if we is not None:
        T, V_inf, V_perp, v_fw, v_ref = we
        print(
            f"  worst |v_err|    = {result['worst_v_err']:.3e} m/s "
            f"at T={T:.2f} N, V_inf={V_inf:+.2f} m/s, V_perp={V_perp:.2f} m/s "
            f"(v_fw={v_fw:.6f}, v_ref={v_ref:.6f})"
        )
    if result["branch_failures"]:
        print(
            f"  branch failures (V_inf+v < 0): {len(result['branch_failures'])} "
            "(against-thrust pirouette, outside model — see plan)"
        )
    else:
        print("  branch condition V_inf+v >= 0 satisfied at every point")


def run_case(problem: MomentumClosure, solvers: list[Solver]) -> None:
    print(
        f"\nCase: T_cmd={problem.T_cmd:.3f} N, "
        f"V_inf={problem.V_inf:.3f} m/s, V_perp={problem.V_perp:.3f} m/s, "
        f"g_m={problem.g_m:.4f}"
    )
    for solver in solvers:
        try:
            t0 = time.perf_counter()
            v = solver.solve(problem)
            dt = time.perf_counter() - t0
            res = problem.residual(v)
            print(
                f"  {solver.name:18s}  v={v:10.5f} m/s  "
                f"|res|={abs(res):.3e}  T_check={problem.thrust(v):.5f} N  "
                f"({dt * 1.0e6:.1f} us)"
            )
        except Exception as exc:
            print(f"  {solver.name:18s}  FAILED: {exc}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Test v_i,ss solvers for tail-rotor momentum closure"
    )
    parser.add_argument("--T", type=float, default=12.0, help="Commanded thrust [N]")
    parser.add_argument("--V-inf", type=float, default=0.0, dest="V_inf")
    parser.add_argument("--V-perp", type=float, default=0.0, dest="V_perp")
    parser.add_argument(
        "--sweep",
        action="store_true",
        help="Sweep the operating envelope and check the firmware Newton solver",
    )
    args = parser.parse_args()

    if args.sweep:
        T_values = [0.5, 1.0, 3.0, 6.0, 12.0, 18.0, 24.0]
        # Smooth-wake / "with-thrust" regime only: V_inf >= 0. Against-thrust
        # pirouettes (V_inf < 0) put the rotor in the windmill-brake or
        # vortex-ring state, where eq. (41) loses uniqueness and the firmware
        # needs an outer guard rather than this solver (§3.3, plan note).
        V_inf_values = [0.0, 0.5, 1.0, 2.0, 4.0, 7.0, 10.0]
        V_perp_values = [0.0, 1.0, 3.0, 6.0, 10.0]
        for n_iter in (1, 2, 3):
            res = sweep_firmware_newton(
                T_values=T_values,
                V_inf_values=V_inf_values,
                V_perp_values=V_perp_values,
                n_iter=n_iter,
            )
            print_sweep_result(res)
        return

    problem = MomentumClosure(T_cmd=args.T, V_inf=args.V_inf, V_perp=args.V_perp)
    run_case(problem, default_solvers())

    if args.V_inf == 0.0 and args.V_perp == 0.0:
        print("\n--- Hover thrust sweep ---")
        hover_solvers = [
            HoverAlgebraicSolver(),
            AxialQuadraticSolver(),
            NewtonSolver(),
            BisectionSolver(),
        ]
        for T_cmd in (0.0, 1.0, 6.0, 12.0, 24.0):
            run_case(MomentumClosure(T_cmd=T_cmd), hover_solvers)

    if args.V_perp == 0.0 and args.V_inf != 0.0:
        print("\n--- Axial V_inf sweep (T_cmd=12 N) ---")
        axial_solvers = [AxialQuadraticSolver(), NewtonSolver(), BisectionSolver()]
        for V_inf in (-2.0, 0.0, 5.0, 10.0):
            run_case(MomentumClosure(T_cmd=12.0, V_inf=V_inf), axial_solvers)


if __name__ == "__main__":
    main()
