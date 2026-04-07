#!/usr/bin/python3
"""
Minimax polynomial approximation via the Remez exchange algorithm.

Computes the best (Chebyshev / L-infinity) polynomial approximation of a given
function on a closed interval, minimising the maximum absolute error.

Usage
-----
    python3 remez.py [options] "<expr>"

    <expr>  Python expression in x, e.g. "sin(x)", "exp(-x**2)", "1/(1+x)"

Options
    -n, --degree   N       Polynomial degree  (default: 5)
    -a             A       Interval left  endpoint  (default: 0.0)
    -b             B       Interval right endpoint  (default: 1.0)
    -s, --samples  N       Dense-grid sample count for extremum search (default: 20000)
    -i, --iter     N       Maximum Remez iterations (default: 100)
    -t, --tol      EPS     Convergence tolerance  (default: 1e-10)
    --parity {even,odd}    Force only even- or odd-degree terms
    --plot                 Show error plot (requires matplotlib)

Examples
    python3 remez.py -n 4 -a 0 -b 1 "sin(x * pi / 2)"
    python3 remez.py -n 5 -a 0 -b 1 --parity odd "sin(x * pi / 2)"
    python3 remez.py -n 6 -a -1 -b 1 "exp(x)"
    python3 remez.py -n 6 -a -1 -b 1 --parity even "cosh(x)"
"""

import sys
import math
import argparse
import numpy as np
from numpy.linalg import solve
from scipy.optimize import minimize_scalar


# ---------------------------------------------------------------------------
# Expression compiler
# ---------------------------------------------------------------------------

_MATH_NS = {k: getattr(math, k) for k in dir(math) if not k.startswith('_')}
_MATH_NS.update({'pi': math.pi, 'e': math.e, 'inf': math.inf})


def make_func(expr: str):
    """Return a callable f(x) from a Python math expression string."""
    code = compile(expr, '<expr>', 'eval')
    def f(x):
        ns = dict(_MATH_NS)
        ns['x'] = x
        return float(eval(code, {'__builtins__': {}}, ns))
    f.__name__ = expr
    return f


# ---------------------------------------------------------------------------
# Polynomial evaluation helpers
# ---------------------------------------------------------------------------

def polyval(coeffs, x):
    """Evaluate polynomial with coefficients [c0, c1, ..., cn] (ascending degree)."""
    result = 0.0
    xpow = 1.0
    for c in coeffs:
        result += c * xpow
        xpow *= x
    return result


def polyval_vec(coeffs, xs):
    """Vectorised version of polyval over a numpy array xs."""
    result = np.zeros_like(xs)
    xpow = np.ones_like(xs)
    for c in coeffs:
        result += c * xpow
        xpow *= xs
    return result


# ---------------------------------------------------------------------------
# Remez algorithm
# ---------------------------------------------------------------------------

def _cheb_extrema(a, b, n):
    """Return n Chebyshev extrema nodes on [a, b], sorted ascending."""
    k = np.arange(n)
    nodes = 0.5*(a + b) + 0.5*(b - a)*np.cos(np.pi*k/(n - 1))
    return np.sort(nodes)


def _active_indices(degree, parity):
    """
    Return the list of active (non-zero) coefficient degrees for the given parity.

    parity=None  → all degrees 0..degree
    parity='even' → even degrees only: 0, 2, 4, ...
    parity='odd'  → odd  degrees only: 1, 3, 5, ...
    """
    if parity is None:
        return list(range(degree + 1))
    start = 0 if parity == 'even' else 1
    return list(range(start, degree + 1, 2))


def _solve_equioscillation(func, refs, active):
    """
    Solve the equioscillation system for a polynomial with given active degrees.

    active  list of m degree indices whose coefficients are free, e.g. [1,3,5].
    refs    sorted array of exactly m+1 reference points.

    Solves the square (m+1)×(m+1) system:

        sum_{j in active} c_j * x_i^j  +  (-1)^i * E  =  f(x_i)
                                                  for i = 0 .. m

    Returns (coeffs, E) where coeffs is a full array indexed by degree
    (zeros at inactive positions) and E is the (signed) equioscillation level.
    """
    m = len(active)           # number of free coefficients
    nref = m + 1              # equioscillation theorem: best approx equioscillates at m+1 points
    assert len(refs) == nref

    A = np.zeros((nref, nref))
    for i, x in enumerate(refs):
        for col, j in enumerate(active):
            A[i, col] = x**j
        A[i, m] = (-1.0)**i   # column for E

    rhs = np.array([func(x) for x in refs])
    sol = solve(A, rhs)       # square system — use direct solver

    degree = max(active)
    coeffs = np.zeros(degree + 1)
    for col, j in enumerate(active):
        coeffs[j] = sol[col]
    return coeffs, float(sol[m])


def _find_extrema(func, coeffs, a, b, n_samples):
    """
    Find all local extrema of err(x) = f(x) - p(x) on [a, b].

    Returns (xs, es) — sorted arrays of extremum positions and error values.
    The endpoints are always included.
    """
    xs_grid = np.linspace(a, b, n_samples)
    fs_grid = np.array([func(x) for x in xs_grid])
    ps_grid = polyval_vec(coeffs, xs_grid)
    err_grid = fs_grid - ps_grid

    xs_out = []
    es_out = []

    # Always include endpoints
    xs_out.append(a)
    es_out.append(err_grid[0])
    xs_out.append(b)
    es_out.append(err_grid[-1])

    # Locate interior extrema via sign changes in the numerical derivative
    derr = np.diff(err_grid)
    sign_change_idx = np.where(np.sign(derr[:-1]) != np.sign(derr[1:]))[0]

    def neg_abs_err(x):
        return -(func(x) - polyval(coeffs, x))**2

    for idx in sign_change_idx:
        lo = xs_grid[idx]
        hi = xs_grid[min(idx + 2, n_samples - 1)]
        if lo >= hi:
            continue
        res = minimize_scalar(neg_abs_err, bounds=(lo, hi), method='bounded',
                              options={'xatol': 1e-14})
        xi = res.x
        ei = func(xi) - polyval(coeffs, xi)
        xs_out.append(xi)
        es_out.append(ei)

    order = np.argsort(xs_out)
    return np.array(xs_out)[order], np.array(es_out)[order]


def _select_refs(xs, es, nref):
    """
    Select nref reference points from (xs, es) for the next Remez iteration.

    Strategy (standard Remez exchange):
    1. Condense consecutive extrema of the same sign — keep the larger one.
    2. If we still have more than nref, drop the smallest-magnitude extremum
       (from either end if at the boundary, otherwise interior) while
       preserving alternation.

    Returns a sorted 1-D array of nref x-positions.
    """
    # Step 1: condense same-sign runs
    c_xs = [xs[0]]
    c_es = [es[0]]
    for xi, ei in zip(xs[1:], es[1:]):
        if np.sign(ei) == np.sign(c_es[-1]):
            # Same sign — keep the larger magnitude
            if abs(ei) > abs(c_es[-1]):
                c_xs[-1] = xi
                c_es[-1] = ei
        else:
            c_xs.append(xi)
            c_es.append(ei)

    c_xs = np.array(c_xs)
    c_es = np.array(c_es)

    # If too few alternating extrema, fall back to Chebyshev nodes
    if len(c_xs) < nref:
        a, b = xs[0], xs[-1]
        return _cheb_extrema(a, b, nref)

    # Step 2: drop smallest-magnitude extrema until we have exactly nref
    while len(c_xs) > nref:
        # Find the extremum with the smallest absolute error; prefer endpoints
        # — removing a non-endpoint is always safe if it preserves alternation.
        magnitudes = np.abs(c_es)
        # Endpoints can only be removed if the sequence still alternates
        # without them and we still have enough points.
        idx_min = int(np.argmin(magnitudes))
        # Remove it and check alternation is preserved
        new_xs = np.delete(c_xs, idx_min)
        new_es = np.delete(c_es, idx_min)
        # Verify alternation after removal
        if _is_alternating(new_es):
            c_xs, c_es = new_xs, new_es
        else:
            # Can't remove that one — try the next smallest
            magnitudes[idx_min] = np.inf
            idx_min2 = int(np.argmin(magnitudes))
            new_xs = np.delete(c_xs, idx_min2)
            new_es = np.delete(c_es, idx_min2)
            if _is_alternating(new_es):
                c_xs, c_es = new_xs, new_es
            else:
                # Give up trimming — shouldn't happen with a well-behaved function
                break

    return c_xs[:nref]


def _is_alternating(es):
    """Return True if es strictly alternates in sign."""
    if len(es) < 2:
        return True
    return all(es[i] * es[i+1] < 0 for i in range(len(es) - 1))


def remez(func, degree, a, b, *, parity=None, n_samples=20000, max_iter=100, tol=1e-10):
    """
    Compute the minimax polynomial approximation of func on [a, b].

    Parameters
    ----------
    func      : callable         f(x) -> float
    degree    : int              polynomial degree (highest included term)
    a, b      : float            approximation interval
    parity    : None|'even'|'odd'  restrict to even- or odd-degree terms only
    n_samples : int              dense-grid points for extremum search
    max_iter  : int              maximum iterations
    tol       : float            convergence tolerance — relative gap between
                                 equioscillation level |E| and true max error

    Returns
    -------
    coeffs    : list[float]   full coefficient array [c0,..,c_degree] (ascending);
                              inactive (parity-forced) positions are zero
    max_err   : float         minimax error bound (supremum of |f - p|)
    info      : dict          convergence diagnostics
    """
    active = _active_indices(degree, parity)
    nref = len(active) + 1     # equioscillation theorem: m free params → m+1 ref points
    refs = _cheb_extrema(a, b, nref)

    coeffs = None
    E = None
    max_err = None
    history = []
    converged = False
    it = 0

    for it in range(1, max_iter + 1):
        try:
            coeffs, E = _solve_equioscillation(func, refs, active)
        except np.linalg.LinAlgError:
            refs += np.random.uniform(-1e-8, 1e-8, size=refs.shape) * (b - a)
            coeffs, E = _solve_equioscillation(func, refs, active)

        abs_E = abs(E)

        xs, es = _find_extrema(func, coeffs, a, b, n_samples)
        max_err = float(np.max(np.abs(es)))

        # Primary convergence: true max error within tol of equioscillation level
        if abs_E > 0 and abs(max_err - abs_E) / abs_E < tol:
            converged = True
            break

        # Secondary: stagnation — |E| unchanged in last 5 steps
        history.append(abs_E)
        if len(history) >= 5:
            recent = history[-5:]
            if abs_E > 0 and (max(recent) - min(recent)) / abs_E < tol:
                converged = True
                break

        refs = _select_refs(xs, es, nref)

    info = {
        'iterations': it,
        'converged': converged,
        'equioscillation_E': E,
        'max_error': max_err,
        'active': active,
        'parity': parity,
    }
    return list(coeffs), max_err, info


# ---------------------------------------------------------------------------
# Output formatting
# ---------------------------------------------------------------------------

def _format_c_float(v, precision=17):
    """Format a float value as a C float literal."""
    s = f'{v:.{precision}g}'
    if 'e' not in s and '.' not in s:
        s += '.0'
    return s + 'f'


def _horner_full(coeffs):
    """Standard Horner form string: c0 + x*(c1 + x*(c2 + ...))."""
    active = [(i, c) for i, c in enumerate(coeffs) if c != 0.0]
    if not active:
        return '0.0f'
    # Build from highest degree downward
    inner = _format_c_float(active[-1][1])
    for i in range(len(active) - 2, -1, -1):
        deg, c = active[i]
        next_deg = active[i + 1][0]
        gap = next_deg - deg
        xterm = 'x' if gap == 1 else f'x*x' if gap == 2 else f'pow(x,{gap})'
        inner = f'{_format_c_float(c)} + {xterm} * ({inner})'
    return inner


def _horner_parity(coeffs, parity):
    """
    Horner form exploiting even/odd structure, evaluating in x² steps.

    Even: c0 + v*(c2 + v*(c4 + ...))            where v = x*x
    Odd:  x * (c1 + v*(c3 + v*(c5 + ...)))      where v = x*x
    """
    active = [(i, c) for i, c in enumerate(coeffs) if c != 0.0]
    if not active:
        return '0.0f'
    # Build Horner in x^2, from highest to lowest
    inner = _format_c_float(active[-1][1])
    for i in range(len(active) - 2, -1, -1):
        inner = f'{_format_c_float(active[i][1])} + v * ({inner})'
    if parity == 'odd':
        return f'x * ({inner})'
    return inner   # even: caller uses v = x*x in surrounding code


def print_results(func_expr, degree, a, b, coeffs, max_err, info):
    parity = info.get('parity')
    active = info.get('active', list(range(degree + 1)))

    width = 72
    print('=' * width)
    print(f'  Minimax polynomial approximation')
    print(f'  f(x) = {func_expr}')
    print(f'  Interval : [{a}, {b}]')
    print(f'  Degree   : {degree}', end='')
    if parity:
        print(f'  ({parity} terms only: {[f"x^{j}" for j in active]})', end='')
    print()
    print('=' * width)

    print(f'\n  Iterations : {info["iterations"]}  '
          f'({"converged" if info["converged"] else "did NOT converge"})')
    print(f'  Equioscillation E : {info["equioscillation_E"]:.6e}')
    print(f'  Max error (actual): {max_err:.6e}')

    print('\n  Coefficients (ascending degree, c0 + c1*x + c2*x^2 + ...):\n')
    for i in active:
        c = coeffs[i]
        print(f'    c[{i}] = {c!r:>26}  /* x^{i} */')

    print('\n  As C float literals:\n')
    for i in active:
        print(f'    /* c[{i}] */  {_format_c_float(coeffs[i])}')

    print('\n  Horner form (C):\n')
    print(f'    {_horner_full(coeffs)}')

    if parity:
        v_decl = '    const float v = x * x;\n'
        print(f'\n  Horner form optimised for {parity} symmetry (v = x*x):\n')
        print(v_decl, end='')
        print(f'    {_horner_parity(coeffs, parity)}')

    print('=' * width)


def plot_error(func, coeffs, a, b, n_samples=4000):
    """Plot f(x) - p(x) over [a, b]."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib not available — skipping plot.')
        return

    xs = np.linspace(a, b, n_samples)
    fs = np.array([func(x) for x in xs])
    ps = polyval_vec(coeffs, xs)
    err = fs - ps

    plt.figure(figsize=(10, 4))
    plt.plot(xs, err, lw=1.2, color='steelblue', label='f(x) − p(x)')
    plt.axhline(0, color='k', lw=0.5)
    plt.axhline( max(abs(err)), color='tomato', lw=0.8, ls='--', label=f'±{max(abs(err)):.3e}')
    plt.axhline(-max(abs(err)), color='tomato', lw=0.8, ls='--')
    plt.xlabel('x')
    plt.ylabel('error')
    plt.title(f'Minimax approximation error  (degree {len(coeffs)-1})')
    plt.legend()
    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Minimax polynomial approximation via the Remez algorithm.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('expr', help='Function expression in x (e.g. "sin(x)")')
    parser.add_argument('-n', '--degree', type=int, default=5,
                        help='Polynomial degree (default: 5)')
    parser.add_argument('-a', type=float, default=0.0,
                        help='Interval left endpoint (default: 0.0)')
    parser.add_argument('-b', type=float, default=1.0,
                        help='Interval right endpoint (default: 1.0)')
    parser.add_argument('-s', '--samples', type=int, default=20000,
                        help='Grid samples for extremum search (default: 20000)')
    parser.add_argument('-i', '--iter', type=int, default=100, dest='max_iter',
                        help='Maximum Remez iterations (default: 100)')
    parser.add_argument('-t', '--tol', type=float, default=1e-10,
                        help='Convergence tolerance (default: 1e-10)')
    parser.add_argument('--parity', choices=['even', 'odd'], default=None,
                        help='Restrict to even- or odd-degree terms only')
    parser.add_argument('--plot', action='store_true',
                        help='Plot the approximation error after convergence')

    args = parser.parse_args()

    if args.a >= args.b:
        print(f'Error: a ({args.a}) must be strictly less than b ({args.b})', file=sys.stderr)
        sys.exit(1)
    if args.degree < 1:
        print('Error: degree must be >= 1', file=sys.stderr)
        sys.exit(1)

    active = _active_indices(args.degree, args.parity)
    if not active:
        print(f'Error: degree {args.degree} with parity={args.parity} yields no active terms.',
              file=sys.stderr)
        sys.exit(1)

    func = make_func(args.expr)

    # Quick sanity check
    try:
        func(0.5 * (args.a + args.b))
    except Exception as exc:
        print(f'Error evaluating expression at midpoint: {exc}', file=sys.stderr)
        sys.exit(1)

    coeffs, max_err, info = remez(
        func, args.degree, args.a, args.b,
        parity=args.parity,
        n_samples=args.samples,
        max_iter=args.max_iter,
        tol=args.tol,
    )

    print_results(args.expr, args.degree, args.a, args.b, coeffs, max_err, info)

    if args.plot:
        plot_error(func, coeffs, args.a, args.b)


if __name__ == '__main__':
    main()
