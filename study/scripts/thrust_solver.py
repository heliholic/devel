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
        NewtonSolver(),
        BisectionSolver(),
    ]
    if brentq is not None:
        solvers.append(BrentqSolver())
    return solvers


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
    args = parser.parse_args()

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
