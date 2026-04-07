#!/usr/bin/python3
"""
Piecewise minimax polynomial approximation with optional C1 continuity.

Partitions [a, b] into N equal-width pieces and fits an independent minimax
polynomial of the given degree on each piece using the Remez algorithm.

Each piece uses a local normalized variable t = (x - x_lo) / h, t ∈ [0, 1],
where h = (b - a) / N.  The coefficient arrays are therefore independent of
the absolute position of each piece.

C1 mode  (--c1):
  The derivative of the spline is continuous at every interior breakpoint.
  Each piece k ≥ 1 has its constant and linear terms fixed by the endpoint
  value and derivative of piece k-1, then solves Remez for degrees 2..D on
  the residual.  Requires degree ≥ 2 (at least one free term per piece).

Usage
-----
    python3 spline.py [options] "<expr>"

    <expr>  Python expression in x, e.g. "sin(x)", "log(1+x)", "1/sqrt(x)"

Options
    -n, --degree   D       Polynomial degree per piece  (default: 3)
    -p, --pieces   N       Number of pieces  (default: 4)
    -a             A       Interval left  endpoint  (default: 0.0)
    -b             B       Interval right endpoint  (default: 1.0)
    -s, --samples  N       Dense-grid sample count per piece (default: 10000)
    -i, --iter     N       Maximum Remez iterations per piece (default: 100)
    -t, --tol      EPS     Convergence tolerance per piece (default: 1e-10)
    --c1                   Enforce C1 continuity at every interior breakpoint
    --name  ID             C identifier prefix for generated code (default: spline)
    --plot                 Show approximation and error plot (requires matplotlib)

Examples
    python3 spline.py -n 3 -p 4 -a 0 -b 1 "sin(x * pi / 2)"
    python3 spline.py -n 3 -p 4 -a 0 -b 1 --c1 "sin(x * pi / 2)"
    python3 spline.py -n 2 -p 8 -a 0 -b 1 --c1 "sqrt(x)"
    python3 spline.py -n 3 -p 8 -a 0 -b 1.5707963 --c1 "sin(x)"
"""

import sys
import math
import argparse
import numpy as np

# Import everything we need from remez.py (same directory)
from remez import (
    remez, make_func, polyval, polyval_vec,
    _format_c_float,
    _solve_equioscillation, _find_extrema, _select_refs, _cheb_extrema,
)


# ---------------------------------------------------------------------------
# Low-level helpers
# ---------------------------------------------------------------------------

def _poly_deriv_at(coeffs, t):
    """Evaluate the derivative of a polynomial (ascending coeffs) at t."""
    result = 0.0
    for j in range(1, len(coeffs)):
        result += j * coeffs[j] * t**(j - 1)
    return result


def _remez_active(func, active, a, b, *, n_samples=10000, max_iter=100, tol=1e-10):
    """
    Remez algorithm with an explicit list of active (free) coefficient degrees.

    Same as remez() but bypasses the degree/parity interface so that callers
    can freely specify which powers of t to solve for — e.g. [2, 3, 4] to
    fix the constant and linear terms and solve only for the higher degrees.

    Returns (coeffs, max_err, info) with the same structure as remez().
    Inactive positions in coeffs are zero.
    """
    nref = len(active) + 1
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

        if abs_E > 0 and abs(max_err - abs_E) / abs_E < tol:
            converged = True
            break

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
    }
    return list(coeffs), max_err, info


# ---------------------------------------------------------------------------
# Piecewise approximation — free (independent pieces)
# ---------------------------------------------------------------------------

def spline_approx(func, n_pieces, degree, a, b, *,
                  n_samples=10000, max_iter=100, tol=1e-10,
                  verbose=True):
    """
    Compute a piecewise minimax polynomial approximation.

    Each piece is solved independently.  No continuity is enforced at the
    breakpoints.

    Returns (pieces, h) where h = (b-a)/n_pieces and pieces is a list of dicts:
        x_lo, x_hi  — boundaries in original coordinates
        coeffs      — polynomial coefficients in t ∈ [0, 1]
        max_err     — minimax error for this piece
        converged   — bool
        iterations  — int
    """
    h = (b - a) / n_pieces
    pieces = []

    for i in range(n_pieces):
        x_lo = a + i * h
        x_hi = a + (i + 1) * h

        def piece_func(t, _lo=x_lo, _h=h):
            return func(_lo + t * _h)

        if verbose:
            print(f'  piece {i:3d}/{n_pieces}  [{x_lo:.6g}, {x_hi:.6g}] ...', end='', flush=True)

        coeffs, max_err, info = remez(
            piece_func, degree, 0.0, 1.0,
            n_samples=n_samples,
            max_iter=max_iter,
            tol=tol,
        )

        if verbose:
            flag = '' if info['converged'] else ' (!)'
            print(f'  max_err = {max_err:.4e}  ({info["iterations"]} iter{flag})')

        pieces.append({
            'index':      i,
            'x_lo':       x_lo,
            'x_hi':       x_hi,
            'coeffs':     coeffs,
            'max_err':    max_err,
            'converged':  info['converged'],
            'iterations': info['iterations'],
        })

    return pieces, h


# ---------------------------------------------------------------------------
# Piecewise approximation — C1 continuous
# ---------------------------------------------------------------------------

def spline_approx_c1(func, n_pieces, degree, a, b, *,
                     n_samples=10000, max_iter=100, tol=1e-10,
                     verbose=True):
    """
    Compute a piecewise minimax polynomial approximation with C1 continuity.

    Piece 0 is solved freely.  For each subsequent piece k, the constant term
    c[0] and linear term c[1] are fixed by the endpoint value and derivative
    of piece k-1:

        c[0]_k = p_{k-1}(1)
        c[1]_k = p'_{k-1}(1)

    Remez is then run on the residual f_k(t) - c[0]_k - c[1]_k * t for
    degrees 2..degree, yielding a polynomial that satisfies both C0 and C1
    at the left boundary by construction.

    Note: the max_err of each constrained piece is the true max error of the
    full polynomial versus the original function, because:

        f_k(t) - full_poly(t)  =  residual(t) - high_poly(t)

    Requires degree >= 2.  parity is not supported in C1 mode.

    Returns (pieces, h) with the same structure as spline_approx().
    """
    if degree < 2:
        raise ValueError(f'C1 mode requires degree >= 2 (got {degree}); '
                         f'each constrained piece needs at least one free term.')

    h = (b - a) / n_pieces
    free_active = list(range(2, degree + 1))   # degrees free on pieces k >= 1

    pieces = []

    for i in range(n_pieces):
        x_lo = a + i * h
        x_hi = a + (i + 1) * h

        def piece_func(t, _lo=x_lo, _h=h):
            return func(_lo + t * _h)

        if verbose:
            print(f'  piece {i:3d}/{n_pieces}  [{x_lo:.6g}, {x_hi:.6g}] ...', end='', flush=True)

        if i == 0:
            # First piece: fully free, all degrees 0..degree
            coeffs, max_err, info = remez(
                piece_func, degree, 0.0, 1.0,
                n_samples=n_samples,
                max_iter=max_iter,
                tol=tol,
            )
        else:
            # Fix c[0] and c[1] from C1 continuity with previous piece
            prev = pieces[-1]['coeffs']
            c0 = polyval(prev, 1.0)          # value at right end of prev piece
            c1 = _poly_deriv_at(prev, 1.0)   # derivative at right end of prev piece

            # Residual: subtract the fixed linear part from the target function.
            # The minimax error of the residual equals the full polynomial error:
            #   f_k(t) - (c0 + c1*t + high(t))  =  residual(t) - high(t)
            def residual(t, _c0=c0, _c1=c1, _pf=piece_func):
                return _pf(t) - _c0 - _c1 * t

            # Solve for degrees 2..degree on the residual
            high_coeffs, max_err, info = _remez_active(
                residual, free_active, 0.0, 1.0,
                n_samples=n_samples,
                max_iter=max_iter,
                tol=tol,
            )

            # Assemble full coefficient array
            coeffs = [0.0] * (degree + 1)
            coeffs[0] = c0
            coeffs[1] = c1
            for j in free_active:
                if j < len(high_coeffs):
                    coeffs[j] = high_coeffs[j]

        if verbose:
            flag = '' if info['converged'] else ' (!)'
            print(f'  max_err = {max_err:.4e}  ({info["iterations"]} iter{flag})')

        pieces.append({
            'index':      i,
            'x_lo':       x_lo,
            'x_hi':       x_hi,
            'coeffs':     coeffs,
            'max_err':    max_err,
            'converged':  info['converged'],
            'iterations': info['iterations'],
        })

    return pieces, h


# ---------------------------------------------------------------------------
# Spline evaluation
# ---------------------------------------------------------------------------

def eval_spline(pieces, h, a, x):
    """Evaluate the piecewise approximation at a single x."""
    n = len(pieces)
    i = int((x - a) / h)
    i = max(0, min(n - 1, i))
    t = (x - pieces[i]['x_lo']) / h
    return polyval(pieces[i]['coeffs'], t)


def eval_spline_vec(pieces, h, a, xs):
    """Vectorised spline evaluation over a numpy array xs."""
    return np.array([eval_spline(pieces, h, a, x) for x in xs])


# ---------------------------------------------------------------------------
# Correct-bits analysis
# ---------------------------------------------------------------------------

def annotate_correct_bits(pieces, func, h, n_samples=2000):
    """
    Add 'correct_bits' to every piece dict.

    Definition
    ----------
    At any point x in a piece the relative error is |f(x) - p(x)| / |f(x)|.
    The worst case (maximum relative error) occurs where |f(x)| is smallest,
    so the guaranteed lower bound on correct mantissa bits is:

        correct_bits = log2( min|f(x)| / max_err )

    This is the number of bits that are *always* correct across the whole
    piece regardless of which x you evaluate.  Float32 has 24 mantissa bits
    (23 stored + 1 implicit), so >= 24 bits means the approximation is exact
    to float32 precision.

    If f has a zero in the piece the relative error is unbounded and
    correct_bits is set to None (reported as "---" in the table).
    """
    for p in pieces:
        x_lo = p['x_lo']
        ts = np.linspace(0.0, 1.0, n_samples)
        abs_fs = np.abs([func(x_lo + t * h) for t in ts])

        f_min = float(np.min(abs_fs))
        max_err = p['max_err']

        if max_err <= 0.0:
            p['correct_bits'] = math.inf
        elif f_min <= 0.0:
            p['correct_bits'] = None        # zero crossing — relative error unbounded
        else:
            p['correct_bits'] = math.log2(f_min / max_err)


# ---------------------------------------------------------------------------
# Output formatting
# ---------------------------------------------------------------------------

def _horner_from_array(degree, var='t', arr='c'):
    """Generate a C Horner evaluation string reading coefficients from arr[]."""
    inner = f'{arr}[{degree}]'
    for j in range(degree - 1, -1, -1):
        inner = f'{arr}[{j}] + {var} * ({inner})'
    return inner


def _check_c1(pieces, h):
    """
    Verify C0 and C1 continuity at every interior breakpoint.

    Returns a list of dicts with keys: breakpoint, c0_gap, c1_gap.
    """
    gaps = []
    for k in range(1, len(pieces)):
        prev = pieces[k - 1]['coeffs']
        cur  = pieces[k]['coeffs']
        c0_gap = abs(polyval(prev, 1.0) - cur[0])
        c1_gap = abs(_poly_deriv_at(prev, 1.0) - (cur[1] if len(cur) > 1 else 0.0))
        gaps.append({
            'breakpoint': pieces[k]['x_lo'],
            'c0_gap':     c0_gap,
            'c1_gap':     c1_gap,
        })
    return gaps


def print_summary(func_expr, n_pieces, degree, a, b, pieces, c1):
    h = (b - a) / n_pieces
    all_errs = [p['max_err'] for p in pieces]
    worst_i = int(np.argmax(all_errs))

    width = 72
    print('=' * width)
    print(f'  Piecewise minimax polynomial approximation')
    print(f'  f(x) = {func_expr}')
    print(f'  Interval   : [{a}, {b}]')
    print(f'  Pieces     : {n_pieces}   h = {h:.6g}')
    print(f'  Degree     : {degree}')
    print(f'  Continuity : {"C1 (value + derivative)" if c1 else "C0 (value only — independent pieces)"}')
    print('=' * width)

    has_bits = 'correct_bits' in pieces[0]
    bits_hdr = f'  {"bits≥":>6}' if has_bits else ''
    print(f'\n  {"Piece":>5}  {"x_lo":>14}  {"x_hi":>14}  {"max_err":>12}  {"iter":>4}  conv{bits_hdr}')
    print(f'  {"-"*5}  {"-"*14}  {"-"*14}  {"-"*12}  {"-"*4}  ----'
          + ('  ------' if has_bits else ''))
    for p in pieces:
        flag = '  *' if p['index'] == worst_i else ''
        conv = 'yes' if p['converged'] else ' NO'
        if has_bits:
            cb = p['correct_bits']
            if cb is None:
                bits_str = f'  {"---":>6}'
            elif math.isinf(cb):
                bits_str = f'  {"inf":>6}'
            else:
                bits_str = f'  {cb:6.1f}'
        else:
            bits_str = ''
        print(f'  {p["index"]:5d}  {p["x_lo"]:14.8g}  {p["x_hi"]:14.8g}'
              f'  {p["max_err"]:12.4e}  {p["iterations"]:4d}  {conv}{bits_str}{flag}')

    print(f'\n  Overall max error : {max(all_errs):.6e}  (worst: piece {worst_i})')
    print(f'  Overall avg error : {np.mean(all_errs):.6e}')

    if c1:
        gaps = _check_c1(pieces, h)
        max_c0 = max(g['c0_gap'] for g in gaps)
        max_c1 = max(g['c1_gap'] for g in gaps)
        print(f'\n  C1 verification (max gap across all breakpoints):')
        print(f'    Value gap      : {max_c0:.3e}  {"OK" if max_c0 < 1e-10 else "WARN"}')
        print(f'    Derivative gap : {max_c1:.3e}  {"OK" if max_c1 < 1e-10 else "WARN"}')


def print_coefficients(pieces, degree):
    print(f'\n  Coefficients per piece (t = (x - x_lo) / h, t ∈ [0, 1]):\n')
    col_w = 22
    header = '  piece' + ''.join(f'  {("c["+str(j)+"]"):>{col_w}}' for j in range(degree + 1))
    print(header)
    print('  ' + '-' * (len(header) - 2))
    for p in pieces:
        row = f'  {p["index"]:5d}'
        for j in range(degree + 1):
            c = p['coeffs'][j] if j < len(p['coeffs']) else 0.0
            row += f'  {c:{col_w}.15g}'
        print(row)


def print_c_code(func_expr, n_pieces, degree, a, b, pieces, c1, name='spline'):
    """Print a self-contained C snippet: coefficient table + inline eval function."""
    h = (b - a) / n_pieces
    inv_h = n_pieces / (b - a)
    n_coeffs = degree + 1
    max_err = max(p['max_err'] for p in pieces)
    NAME = name.upper()

    print(f'\n  C code  (prefix: {name}_):\n')
    print('  ' + '-' * 68)

    cont = ',  C1 continuous' if c1 else ''
    lines = [
        f'/* Piecewise minimax: f(x) = {func_expr} */',
        f'/* [{a}, {b}],  N={n_pieces} pieces,  degree={degree}{cont},  max_err={max_err:.4e} */',
        f'',
        f'#define {NAME}_N       {n_pieces}',
        f'#define {NAME}_A       {_format_c_float(a)}',
        f'#define {NAME}_B       {_format_c_float(b)}',
        f'#define {NAME}_H       {_format_c_float(h)}',
        f'#define {NAME}_INV_H   {_format_c_float(inv_h)}',
        f'',
        f'static const float {name}_coeffs[{NAME}_N][{n_coeffs}] = {{',
    ]

    for p in pieces:
        vals = ', '.join(
            _format_c_float(p['coeffs'][j] if j < len(p['coeffs']) else 0.0)
            for j in range(n_coeffs)
        )
        lines.append(f'    {{ {vals} }},  /* piece {p["index"]}'
                     f'  [{p["x_lo"]:.6g}, {p["x_hi"]:.6g}]'
                     f'  err={p["max_err"]:.3e} */')

    lines += [
        '};',
        '',
        f'static inline float {name}_eval(float x)',
        '{',
        f'    int i = (int)((x - {NAME}_A) * {NAME}_INV_H);',
        f'    if (i < 0) i = 0;',
        f'    if (i >= {NAME}_N) i = {NAME}_N - 1;',
        f'    const float t = (x - ({NAME}_A + i * {NAME}_H)) * {NAME}_INV_H;',
        f'    const float *c = {name}_coeffs[i];',
        f'    return {_horner_from_array(degree)};',
        '}',
    ]

    for line in lines:
        print(f'  {line}')
    print('  ' + '-' * 68)


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------

def plot_spline(func, pieces, h, a, b, func_expr, c1, n_samples=8000):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib not available — skipping plot.')
        return

    xs = np.linspace(a, b, n_samples)
    fs = np.array([func(x) for x in xs])
    ps = eval_spline_vec(pieces, h, a, xs)
    err = fs - ps

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 6), sharex=True)

    ax1.plot(xs, fs, lw=1.0, color='steelblue', label='f(x)')
    ax1.plot(xs, ps, lw=1.0, color='tomato', ls='--', label='spline(x)')
    for p in pieces:
        ax1.axvline(p['x_lo'], color='gray', lw=0.4, ls=':')
    ax1.axvline(b, color='gray', lw=0.4, ls=':')
    ax1.set_ylabel('value')
    ax1.legend(fontsize=8)
    cont_label = 'C1' if c1 else 'C0'
    ax1.set_title(f'Piecewise minimax ({cont_label})  —  f(x) = {func_expr}  '
                  f'(N={len(pieces)}, degree={degree_of(pieces)})')

    ax2.plot(xs, err, lw=1.0, color='steelblue', label='f(x) − spline(x)')
    ax2.axhline(0, color='k', lw=0.5)
    peak = float(np.max(np.abs(err)))
    ax2.axhline( peak, color='tomato', lw=0.8, ls='--', label=f'±{peak:.3e}')
    ax2.axhline(-peak, color='tomato', lw=0.8, ls='--')
    for p in pieces:
        ax2.axvline(p['x_lo'], color='gray', lw=0.4, ls=':')
    ax2.axvline(b, color='gray', lw=0.4, ls=':')
    ax2.set_xlabel('x')
    ax2.set_ylabel('error')
    ax2.legend(fontsize=8)

    plt.tight_layout()
    plt.show()


def degree_of(pieces):
    return len(pieces[0]['coeffs']) - 1


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Piecewise minimax polynomial approximation.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('expr', help='Function expression in x')
    parser.add_argument('-n', '--degree', type=int, default=3,
                        help='Polynomial degree per piece (default: 3)')
    parser.add_argument('-p', '--pieces', type=int, default=4,
                        help='Number of equal-width pieces (default: 4)')
    parser.add_argument('-a', type=float, default=0.0,
                        help='Interval left endpoint (default: 0.0)')
    parser.add_argument('-b', type=float, default=1.0,
                        help='Interval right endpoint (default: 1.0)')
    parser.add_argument('-s', '--samples', type=int, default=10000,
                        help='Grid samples per piece for extremum search (default: 10000)')
    parser.add_argument('-i', '--iter', type=int, default=100, dest='max_iter',
                        help='Maximum Remez iterations per piece (default: 100)')
    parser.add_argument('-t', '--tol', type=float, default=1e-10,
                        help='Convergence tolerance per piece (default: 1e-10)')
    parser.add_argument('--c1', action='store_true',
                        help='Enforce C1 continuity at every interior breakpoint')
    parser.add_argument('--name', default='spline',
                        help='C identifier prefix for generated code (default: spline)')
    parser.add_argument('--plot', action='store_true',
                        help='Show approximation and error plot (requires matplotlib)')

    args = parser.parse_args()

    if args.a >= args.b:
        print(f'Error: a ({args.a}) must be strictly less than b ({args.b})', file=sys.stderr)
        sys.exit(1)
    if args.degree < 1:
        print('Error: degree must be >= 1', file=sys.stderr)
        sys.exit(1)
    if args.pieces < 1:
        print('Error: pieces must be >= 1', file=sys.stderr)
        sys.exit(1)
    if args.c1 and args.degree < 2:
        print('Error: --c1 requires degree >= 2 (need at least one free term per constrained piece)',
              file=sys.stderr)
        sys.exit(1)
    func = make_func(args.expr)
    try:
        func(0.5 * (args.a + args.b))
    except Exception as exc:
        print(f'Error evaluating expression at midpoint: {exc}', file=sys.stderr)
        sys.exit(1)

    cont_label = 'C1' if args.c1 else 'C0'
    print(f'\nApproximating f(x) = {args.expr}')
    print(f'Interval [{args.a}, {args.b}],  {args.pieces} pieces,  degree {args.degree},  {cont_label}\n')

    if args.c1:
        pieces, h = spline_approx_c1(
            func, args.pieces, args.degree, args.a, args.b,
            n_samples=args.samples,
            max_iter=args.max_iter,
            tol=args.tol,
            verbose=True,
        )
    else:
        pieces, h = spline_approx(
            func, args.pieces, args.degree, args.a, args.b,
            n_samples=args.samples,
            max_iter=args.max_iter,
            tol=args.tol,
            verbose=True,
        )

    annotate_correct_bits(pieces, func, h)

    print()
    print_summary(args.expr, args.pieces, args.degree, args.a, args.b,
                  pieces, args.c1)
    print_coefficients(pieces, args.degree)
    print_c_code(args.expr, args.pieces, args.degree, args.a, args.b,
                 pieces, args.c1, name=args.name)

    if args.plot:
        plot_spline(func, pieces, h, args.a, args.b, args.expr, args.c1)


if __name__ == '__main__':
    main()
