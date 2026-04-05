#!/usr/bin/python3

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from filter import NotchFilter, SVFNotch

import numpy as np
import matplotlib.pyplot as plt


def test1() -> None:
    sample_rate_hz = 8000.0
    duration_s = 1.0

    tone_hz = 201
    notch_hz = 200.0
    notch_q = 3.0

    n = int(sample_rate_hz * duration_s)
    fs = sample_rate_hz
    t = np.arange(n, dtype=np.float32) / fs
    x = np.sin(2.0 * np.pi * tone_hz * t)

    sos_notch = NotchFilter(notch_hz, notch_q, sample_rate_hz)
    svf_notch = SVFNotch(notch_hz, sample_rate_hz, notch_q)

    y_sos = np.empty(n, dtype=np.float32)
    y_svf = np.empty(n, dtype=np.float32)
    for i, s in enumerate(x):
        y_sos[i] = sos_notch.apply(s)
        y_svf[i] = svf_notch.apply(s)

    # Higher figure.dpi scales line width and text in pixels (pt × dpi/72) for a thicker, larger plot.
    fig, ax = plt.subplots(figsize=(10, 5), dpi=200)

    ax.plot(t, x, label=f"input: {tone_hz:g} Hz sine", alpha=0.85)
    ax.plot(t, y_sos, label=f"biquad notch f0={notch_hz:g} Hz, Q={notch_q:g}", alpha=0.85)
    ax.plot(t, y_svf, label=f"SVFNotch f0={notch_hz:g} Hz, Q={notch_q:g}", alpha=0.85)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("amplitude")
    ax.set_title(
        f"{tone_hz:g} Hz sine, {sample_rate_hz:g} Hz sample rate, {duration_s:g} s; "
        f"notch at {notch_hz:g} Hz"
    )
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    plt.show()


def test2() -> None:
    sample_rate_hz = 8000.0
    duration_s = 1.0

    tone_hz_before = 100.0
    tone_hz_after = 150.0
    tone_switch_s = 0.5
    notch_hz = 100
    notch_q = 5.0

    n = int(sample_rate_hz * duration_s)
    fs = sample_rate_hz
    t = np.arange(n, dtype=np.float32) / fs
    x = np.empty(n, dtype=np.float32)

    m0 = t < tone_switch_s
    x[m0] = np.sin(2.0 * np.pi * tone_hz_before * t[m0])
    x[~m0] = np.sin(2.0 * np.pi * tone_hz_after * (t[~m0] - tone_switch_s))

    sos_notch = NotchFilter(notch_hz, notch_q, sample_rate_hz)
    svf_notch = SVFNotch(notch_hz, sample_rate_hz, notch_q)

    y_sos = np.empty(n, dtype=np.float32)
    y_svf = np.empty(n, dtype=np.float32)
    for i, s in enumerate(x):
        y_sos[i] = sos_notch.apply(s)
        y_svf[i] = svf_notch.apply(s)

    # Higher figure.dpi scales line width and text in pixels (pt × dpi/72) for a thicker, larger plot.
    fig, ax = plt.subplots(figsize=(10, 5), dpi=200)

    ax.plot(
        t,
        x,
        label=f"input: {tone_hz_before:g} Hz → {tone_hz_after:g} Hz @ t={tone_switch_s:g} s",
        alpha=0.85,
    )
    ax.plot(t, y_sos, label=f"biquad notch f0={notch_hz:g} Hz, Q={notch_q:g}", alpha=0.85)
    ax.plot(t, y_svf, label=f"SVFNotch f0={notch_hz:g} Hz, Q={notch_q:g}", alpha=0.85)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("amplitude")
    ax.set_title(
        f"{tone_hz_before:g} / {tone_hz_after:g} Hz sine, {sample_rate_hz:g} Hz SR, {duration_s:g} s; "
        f"notch at {notch_hz:g} Hz"
    )
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    plt.show()


def test3() -> None:
    sample_rate_hz = 8000.0
    duration_s = 1.0

    tone_hz = 100.0
    notch_hz_lo = 80.0
    notch_hz_hi = 120.0
    notch_q = 5.0

    n = int(sample_rate_hz * duration_s)
    fs = sample_rate_hz
    t = np.arange(n, dtype=np.float32) / fs
    x = np.sin(2.0 * np.pi * tone_hz * t)

    sos_notch = NotchFilter(notch_hz_lo, notch_q, sample_rate_hz)
    svf_notch = SVFNotch(notch_hz_lo, sample_rate_hz, notch_q)

    y_sos = np.empty(n, dtype=np.float32)
    y_svf = np.empty(n, dtype=np.float32)
    for i in range(n):
        ti = float(t[i])
        notch_hz = notch_hz_lo + (notch_hz_hi - notch_hz_lo) * (ti / duration_s)
        sos_notch.update_notch(notch_hz, notch_q, sample_rate_hz)
        svf_notch.update_svf(notch_hz, notch_q, sample_rate_hz)
        y_sos[i] = sos_notch.apply(float(x[i]))
        y_svf[i] = svf_notch.apply(float(x[i]))

    # Higher figure.dpi scales line width and text in pixels (pt × dpi/72) for a thicker, larger plot.
    fig, ax = plt.subplots(figsize=(10, 5), dpi=200)

    ax.plot(t, x, label=f"input: {tone_hz:g} Hz sine", alpha=0.85)
    ax.plot(
        t,
        y_sos,
        label=f"biquad notch f0 {notch_hz_lo:g}→{notch_hz_hi:g} Hz, Q={notch_q:g}",
        alpha=0.85,
    )
    ax.plot(
        t,
        y_svf,
        label=f"SVFNotch f0 {notch_hz_lo:g}→{notch_hz_hi:g} Hz, Q={notch_q:g}",
        alpha=0.85,
    )
    ax.set_xlabel("time (s)")
    ax.set_ylabel("amplitude")
    ax.set_title(
        f"{tone_hz:g} Hz sine, {sample_rate_hz:g} Hz SR, {duration_s:g} s; "
        f"notch f0 swept {notch_hz_lo:g}→{notch_hz_hi:g} Hz"
    )
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    plt.show()


TESTS = [test1, test2, test3]


def main() -> None:
    prog = Path(sys.argv[0]).name
    if len(sys.argv) == 1:
        for i, test_fn in enumerate(TESTS, start=1):
            print(f"=== test {i} ({test_fn.__name__}) ===", flush=True)
            test_fn()
        return

    try:
        idx = int(sys.argv[1], 10)
    except ValueError:
        print(f"usage: {prog} [N]", file=sys.stderr)
        print(f"  run all tests with no arguments, or test N (1..{len(TESTS)})", file=sys.stderr)
        sys.exit(2)

    if idx < 1 or idx > len(TESTS):
        print(f"unknown test: {idx} (valid: 1..{len(TESTS)})", file=sys.stderr)
        sys.exit(2)

    TESTS[idx - 1]()


if __name__ == "__main__":
    main()
