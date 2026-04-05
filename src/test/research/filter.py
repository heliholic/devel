#!/usr/bin/env python3
"""
Python mirror of src/main/common/filter.c for research / simulation.

Uses math.{tan,sin,cos} where the firmware uses tan_approx / sin_approx / cos_approx.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from enum import IntEnum

import math

# --- Constants from filter.h / maths.h ---

M_1_2PI = 0.15915494309189533577
M_PI = math.pi
M_2PI = 2.0 * math.pi

BUTTER_Q = 0.707106781
BESSEL_Q = 0.577350269
DAMPED_Q = 0.5

BUTTER_C = 1.0
BESSEL_C = 1.272019649
DAMPED_C = 1.553773974

PT2_CUTOFF_SCALE = 1.553773974
PT3_CUTOFF_SCALE = 1.961459177


def constrainf(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


class BiquadKind(IntEnum):
    """Which biquad mode ``SOSFilter`` / firmware biquad uses (LPF, HPF, BPF, notch)."""

    NULL = 0
    LPF = 1
    HPF = 2
    BPF = 3
    NOTCH = 4
    COUNT = 5  # BIQUAD_COUNT


class BaseFilter(ABC):
    """Base for single-step filters: ``update`` (parameters), ``apply`` (samples), ``output`` (state)."""

    @staticmethod
    def limit_cutoff(cutoff: float, sample_rate: float) -> float:
        """95% of Nyquist."""
        return min(cutoff, 0.475 * sample_rate)

    @abstractmethod
    def update(self, cutoff: float, sample_rate: float) -> None:
        """Recompute coefficients from cutoff and sample rate."""

    @abstractmethod
    def apply(self, x: float) -> float:
        """Advance one sample; return filtered output."""

    @property
    def output(self) -> float:
        """Last output."""
        raise NotImplementedError


# --- NIL ---

class NilFilter(BaseFilter):
    """Passthrough: output equals input; no dynamics."""

    def __init__(self) -> None:
        self._y1 = 0.0

    def init(self, cutoff: float = 0.0, sample_rate: float = 0.0) -> None:
        del cutoff, sample_rate
        self._y1 = 0.0

    def update(self, cutoff: float, sample_rate: float) -> None:
        del cutoff, sample_rate

    def apply(self, x: float) -> float:
        self._y1 = x
        return self._y1

    @property
    def output(self) -> float:
        return self._y1


# --- PT1 / PT2 / PT3 ---


class PT1Filter(BaseFilter):
    """First-order (one-pole) lowpass; ``pt1*`` in filter.c."""

    def __init__(self, cutoff: float, sample_rate: float) -> None:
        self._y1 = 0.0
        self._gain = self.filter_gain(cutoff, sample_rate)

    @staticmethod
    def filter_gain(cutoff: float, sample_rate: float) -> float:
        if cutoff > 0 and sample_rate > 0:
            c = BaseFilter.limit_cutoff(cutoff, sample_rate)
            gamma = M_1_2PI * sample_rate
            alpha = c / (c + gamma)
            return min(alpha, 1.0)
        return 1.0

    @classmethod
    def from_gain(cls, gain: float) -> "PT1Filter":
        f = cls.__new__(cls)
        f._y1 = 0.0
        f._gain = gain
        return f

    def init(self, cutoff: float, sample_rate: float) -> None:
        self._y1 = 0.0
        self._gain = self.filter_gain(cutoff, sample_rate)

    def update(self, cutoff: float, sample_rate: float) -> None:
        self._gain = self.filter_gain(cutoff, sample_rate)

    def update_gain(self, gain: float) -> None:
        self._gain = gain

    def apply(self, x: float) -> float:
        self._y1 += (x - self._y1) * self._gain
        return self._y1

    @property
    def output(self) -> float:
        return self._y1


class PT2Filter(BaseFilter):
    """Second-order lowpass as two cascaded PT1 stages; ``pt2*`` in filter.c."""

    def __init__(self, cutoff: float, sample_rate: float) -> None:
        self._y1 = 0.0
        self._y2 = 0.0
        self._gain = self.filter_gain(cutoff, sample_rate)

    @staticmethod
    def filter_gain(cutoff: float, sample_rate: float) -> float:
        return PT1Filter.filter_gain(cutoff * PT2_CUTOFF_SCALE, sample_rate)

    @classmethod
    def from_gain(cls, gain: float) -> "PT2Filter":
        f = cls.__new__(cls)
        f._y1 = f._y2 = 0.0
        f._gain = gain
        return f

    def init(self, cutoff: float, sample_rate: float) -> None:
        self._y1 = self._y2 = 0.0
        self._gain = self.filter_gain(cutoff, sample_rate)

    def update(self, cutoff: float, sample_rate: float) -> None:
        self._gain = self.filter_gain(cutoff, sample_rate)

    def update_gain(self, gain: float) -> None:
        self._gain = gain

    def apply(self, x: float) -> float:
        self._y2 += (x - self._y2) * self._gain
        self._y1 += (self._y2 - self._y1) * self._gain
        return self._y1

    @property
    def output(self) -> float:
        return self._y1


class PT3Filter(BaseFilter):
    """Third-order lowpass as three cascaded PT1 stages; ``pt3*`` in filter.c."""

    def __init__(self, cutoff: float, sample_rate: float) -> None:
        self._y1 = self._y2 = self._y3 = 0.0
        self._gain = self.filter_gain(cutoff, sample_rate)

    @staticmethod
    def filter_gain(cutoff: float, sample_rate: float) -> float:
        return PT1Filter.filter_gain(cutoff * PT3_CUTOFF_SCALE, sample_rate)

    @classmethod
    def from_gain(cls, gain: float) -> "PT3Filter":
        f = cls.__new__(cls)
        f._y1 = f._y2 = f._y3 = 0.0
        f._gain = gain
        return f

    def init(self, cutoff: float, sample_rate: float) -> None:
        self._y1 = self._y2 = self._y3 = 0.0
        self._gain = self.filter_gain(cutoff, sample_rate)

    def update(self, cutoff: float, sample_rate: float) -> None:
        self._gain = self.filter_gain(cutoff, sample_rate)

    def update_gain(self, gain: float) -> None:
        self._gain = gain

    def apply(self, x: float) -> float:
        self._y3 += (x - self._y3) * self._gain
        self._y2 += (self._y3 - self._y2) * self._gain
        self._y1 += (self._y2 - self._y1) * self._gain
        return self._y1

    @property
    def output(self) -> float:
        return self._y1


# --- EWMA1 / EWMA2 / EWMA3 ---


class EWMA1Filter(BaseFilter):
    """Single-section exponentially weighted moving average with finite startup ramp."""

    def __init__(self, cutoff: float, sample_rate: float) -> None:
        self._y1 = 0.0
        self._n = 0
        self._W = self.filter_weight(cutoff, sample_rate)

    @staticmethod
    def filter_weight(cutoff: float, sample_rate: float) -> float:
        if cutoff > 0 and sample_rate > 0:
            c = BaseFilter.limit_cutoff(cutoff, sample_rate)
            gamma = M_1_2PI * sample_rate
            weight = (c + gamma) / c
            return max(weight, 1.0)
        return 1.0

    @classmethod
    def from_weight(cls, weight: float) -> "EWMA1Filter":
        f = cls.__new__(cls)
        f._y1 = 0.0
        f._n = 0
        f._W = weight
        return f

    def init(self, cutoff: float, sample_rate: float) -> None:
        self._y1 = 0.0
        self._n = 0
        self._W = self.filter_weight(cutoff, sample_rate)

    def update(self, cutoff: float, sample_rate: float) -> None:
        self._W = self.filter_weight(cutoff, sample_rate)
        self._n = int(min(float(self._n), self._W))

    def update_weight(self, weight: float) -> None:
        self._W = weight
        self._n = int(min(float(self._n), self._W))

    def apply(self, x: float) -> float:
        count = self._n + 1
        weight = self._W
        if count < weight:
            weight = float(count)
            self._n = count
        self._y1 += (x - self._y1) / weight
        return self._y1

    @property
    def output(self) -> float:
        return self._y1


class EWMA2Filter(BaseFilter):
    """Two cascaded EWMA sections (2nd-order smoothing)."""

    def __init__(self, cutoff: float, sample_rate: float) -> None:
        self._y1 = self._y2 = 0.0
        self._n = 0
        self._W = self.filter_weight(cutoff, sample_rate)

    @staticmethod
    def filter_weight(cutoff: float, sample_rate: float) -> float:
        return EWMA1Filter.filter_weight(cutoff * PT2_CUTOFF_SCALE, sample_rate)

    @classmethod
    def from_weight(cls, weight: float) -> "EWMA2Filter":
        f = cls.__new__(cls)
        f._y1 = f._y2 = 0.0
        f._n = 0
        f._W = weight
        return f

    def init(self, cutoff: float, sample_rate: float) -> None:
        self._y1 = self._y2 = 0.0
        self._n = 0
        self._W = self.filter_weight(cutoff, sample_rate)

    def update(self, cutoff: float, sample_rate: float) -> None:
        self._W = self.filter_weight(cutoff, sample_rate)
        self._n = int(min(float(self._n), self._W))

    def update_weight(self, weight: float) -> None:
        self._W = weight
        self._n = int(min(float(self._n), self._W))

    def apply(self, x: float) -> float:
        count = self._n + 1
        weight = self._W
        if count < weight:
            weight = float(count)
            self._n = count
        self._y2 += (x - self._y2) / weight
        self._y1 += (self._y2 - self._y1) / weight
        return self._y1

    @property
    def output(self) -> float:
        return self._y1


class EWMA3Filter(BaseFilter):
    """Three cascaded EWMA sections (3rd-order smoothing)."""

    def __init__(self, cutoff: float, sample_rate: float) -> None:
        self._y1 = self._y2 = self._y3 = 0.0
        self._n = 0
        self._W = self.filter_weight(cutoff, sample_rate)

    @staticmethod
    def filter_weight(cutoff: float, sample_rate: float) -> float:
        return EWMA1Filter.filter_weight(cutoff * PT3_CUTOFF_SCALE, sample_rate)

    @classmethod
    def from_weight(cls, weight: float) -> "EWMA3Filter":
        f = cls.__new__(cls)
        f._y1 = f._y2 = f._y3 = 0.0
        f._n = 0
        f._W = weight
        return f

    def init(self, cutoff: float, sample_rate: float) -> None:
        self._y1 = self._y2 = self._y3 = 0.0
        self._n = 0
        self._W = self.filter_weight(cutoff, sample_rate)

    def update(self, cutoff: float, sample_rate: float) -> None:
        self._W = self.filter_weight(cutoff, sample_rate)
        self._n = int(min(float(self._n), self._W))

    def update_weight(self, weight: float) -> None:
        self._W = weight
        self._n = int(min(float(self._n), self._W))

    def apply(self, x: float) -> float:
        count = self._n + 1
        weight = self._W
        if count < weight:
            weight = float(count)
            self._n = count
        self._y3 += (x - self._y3) / weight
        self._y2 += (self._y3 - self._y2) / weight
        self._y1 += (self._y2 - self._y1) / weight
        return self._y1

    @property
    def output(self) -> float:
        return self._y1


# --- Differentiator ---

class DifferentiatorFilter(BaseFilter):
    """Discrete-time differentiator (high-frequency gain) with bilinear damping."""

    def __init__(self, cutoff: float, sample_rate: float) -> None:
        self._x1 = 0.0
        self._y1 = 0.0
        self._a = 0.0
        self._b = 0.0
        self.update(cutoff, sample_rate)

    def update(self, cutoff: float, sample_rate: float) -> None:
        if cutoff > 0 and sample_rate > 0:
            c = BaseFilter.limit_cutoff(cutoff, sample_rate / 2.0)
            w = math.tan(M_PI * c / sample_rate)
            self._a = (w - 1.0) / (w + 1.0)
            self._b = 2.0 * sample_rate * w / (w + 1.0)
        else:
            self._a = 0.0
            self._b = 0.0

    def apply(self, x: float) -> float:
        out = self._b * (x - self._x1) - self._a * self._y1
        self._x1 = x
        self._y1 = out
        return out

    @property
    def output(self) -> float:
        return self._y1


# --- Bilinear integrator ---

class IntegratorFilter(BaseFilter):
    """Bilinear integrator with clamped output (anti-windup style limits)."""

    def __init__(self, sample_rate: float, min_out: float, max_out: float) -> None:
        self._x1 = 0.0
        self._y1 = 0.0
        self._min = min_out
        self._max = max_out
        self._gain = 0.0
        self.update(0.0, sample_rate)

    def reset(self) -> None:
        self._x1 = 0.0
        self._y1 = 0.0

    def update(self, cutoff: float, sample_rate: float) -> None:
        del cutoff
        if sample_rate > 0:
            self._gain = 1.0 / (2.0 * sample_rate)
        else:
            self._gain = 0.0

    def update_limits(self, min_out: float, max_out: float) -> None:
        self._min = min_out
        self._max = max_out

    def apply(self, x: float) -> float:
        out = self._y1 + (x + self._x1) * self._gain
        out = constrainf(out, self._min, self._max)
        self._x1 = x
        self._y1 = out
        return out

    @property
    def output(self) -> float:
        return self._y1



# --- First-order bilinear LPF / HPF (shared state layout) ---


class Order1Filter(BaseFilter):
    """Shared first-order IIR state and ``apply``; subclasses must implement ``update(cutoff, sample_rate)``."""

    def __init__(self) -> None:
        self._x1 = 0.0
        self._y1 = 0.0
        self._b0 = 1.0
        self._b1 = 0.0
        self._a1 = 0.0

    def apply(self, x: float) -> float:
        out = self._b0 * x + self._b1 * self._x1 - self._a1 * self._y1
        self._x1 = x
        self._y1 = out
        return out

    @property
    def output(self) -> float:
        return self._y1


class FirstOrderLPF(Order1Filter):
    """First-order bilinear lowpass; ``firstOrderLPF*`` in filter.c."""

    def __init__(self, cutoff: float, sample_rate: float) -> None:
        super().__init__()
        self.init(cutoff, sample_rate)

    def init(self, cutoff: float, sample_rate: float) -> None:
        self._x1 = self._y1 = 0.0
        self.update(cutoff, sample_rate)

    def update(self, cutoff: float, sample_rate: float) -> None:
        if cutoff > 0 and sample_rate > 0:
            c = BaseFilter.limit_cutoff(cutoff, sample_rate)
            w = math.tan(M_PI * c / sample_rate)
            self._a1 = (w - 1.0) / (w + 1.0)
            self._b0 = w / (w + 1.0)
            self._b1 = self._b0
        else:
            self._b0 = 1.0
            self._b1 = 0.0
            self._a1 = 0.0


class FirstOrderHPF(Order1Filter):
    """First-order bilinear highpass; ``firstOrderHPF*`` in filter.c."""

    def __init__(self, cutoff: float, sample_rate: float) -> None:
        super().__init__()
        self.init(cutoff, sample_rate)

    def init(self, cutoff: float, sample_rate: float) -> None:
        self._x1 = self._y1 = 0.0
        self.update(cutoff, sample_rate)

    def update(self, cutoff: float, sample_rate: float) -> None:
        if cutoff > 0 and sample_rate > 0:
            c = BaseFilter.limit_cutoff(cutoff, sample_rate / 2.0)
            w = math.tan(M_PI * c / sample_rate)
            self._a1 = (w - 1.0) / (w + 1.0)
            self._b0 = 1.0 / (w + 1.0)
            self._b1 = -self._b0
        else:
            self._b0 = 1.0
            self._b1 = 0.0
            self._a1 = 0.0


# --- Biquad (SOS) ---

class SOSFilter(BaseFilter):
    """Second-order IIR biquad (one section); direct form II transposed in ``apply``."""

    def __init__(
        self,
        cutoff: float,
        sample_rate: float,
        q: float,
        kind: BiquadKind,
    ) -> None:
        self._y1 = self._y2 = 0.0
        self._x1 = self._x2 = 0.0
        self._b0 = self._b1 = self._b2 = 0.0
        self._a1 = self._a2 = 0.0
        self._q = q
        self._kind = kind
        self.update_sos(cutoff, sample_rate, q, kind)

    def update(self, cutoff: float, sample_rate: float) -> None:
        self.update_sos(cutoff, sample_rate, self._q, self._kind)

    def update_sos(
        self,
        cutoff: float,
        sample_rate: float,
        q: float,
        kind: BiquadKind,
    ) -> None:
        self._q = q
        self._kind = kind
        if (
            cutoff > 0
            and sample_rate > 0
            and q > 0
            and BiquadKind.NULL < kind < BiquadKind.COUNT
        ):
            c = BaseFilter.limit_cutoff(cutoff, sample_rate)
            omega = M_2PI * c / sample_rate
            sinom = math.sin(omega)
            cosom = math.cos(omega)
            alpha = sinom / (2.0 * q)

            if kind == BiquadKind.LPF:
                b1 = 1.0 - cosom
                b0 = b1 / 2.0
                b2 = b0
                a1 = -2.0 * cosom
                a2 = 1.0 - alpha
            elif kind == BiquadKind.HPF:
                b0 = (1.0 + cosom) / 2.0
                b1 = -1.0 - cosom
                b2 = b0
                a1 = -2.0 * cosom
                a2 = 1.0 - alpha
            elif kind == BiquadKind.BPF:
                b0 = alpha
                b1 = 0.0
                b2 = -alpha
                a1 = -2.0 * cosom
                a2 = 1.0 - alpha
            elif kind == BiquadKind.NOTCH:
                b0 = 1.0
                b1 = -2.0 * cosom
                b2 = 1.0
                a1 = b1
                a2 = 1.0 - alpha
            else:
                self._set_identity()
                return

            a0 = 1.0 + alpha
            self._b0 = b0 / a0
            self._b1 = b1 / a0
            self._b2 = b2 / a0
            self._a1 = a1 / a0
            self._a2 = a2 / a0
        else:
            self._set_identity()

    def _set_identity(self) -> None:
        self._b0 = 1.0
        self._b1 = self._b2 = self._a1 = self._a2 = 0.0

    def apply_df1(self, x: float) -> float:
        out = (
            self._b0 * x
            + self._b1 * self._x1
            + self._b2 * self._x2
            - self._a1 * self._y1
            - self._a2 * self._y2
        )
        self._x2 = self._x1
        self._x1 = x
        self._y2 = self._y1
        self._y1 = out
        return out

    def apply_tf2(self, x: float) -> float:
        out = self._b0 * x + self._x1
        self._x1 = self._b1 * x - self._a1 * out + self._x2
        self._x2 = self._b2 * x - self._a2 * out
        self._y1 = out
        return out

    def apply(self, x: float) -> float:
        return self.apply_tf2(x)

    @property
    def output(self) -> float:
        return self._y1


class ButterworthLPF(SOSFilter):
    """Butterworth second-order lowpass (scaled cutoff, classic Q)."""

    def __init__(self, cutoff: float, sample_rate: float) -> None:
        super().__init__(BUTTER_C * cutoff, sample_rate, BUTTER_Q, BiquadKind.LPF)

    def update(self, cutoff: float, sample_rate: float) -> None:
        self.update_sos(BUTTER_C * cutoff, sample_rate, BUTTER_Q, BiquadKind.LPF)


class BesselLPF(SOSFilter):
    """Bessel second-order lowpass (maximally flat group delay)."""

    def __init__(self, cutoff: float, sample_rate: float) -> None:
        super().__init__(BESSEL_C * cutoff, sample_rate, BESSEL_Q, BiquadKind.LPF)

    def update(self, cutoff: float, sample_rate: float) -> None:
        self.update_sos(BESSEL_C * cutoff, sample_rate, BESSEL_Q, BiquadKind.LPF)


class DampedLPF(SOSFilter):
    """Damped second-order lowpass (scaled cutoff, lower Q than Butterworth)."""

    def __init__(self, cutoff: float, sample_rate: float) -> None:
        super().__init__(DAMPED_C * cutoff, sample_rate, DAMPED_Q, BiquadKind.LPF)

    def update(self, cutoff: float, sample_rate: float) -> None:
        self.update_sos(DAMPED_C * cutoff, sample_rate, DAMPED_Q, BiquadKind.LPF)


class NotchFilter(SOSFilter):
    """Second-order notch (band reject) at ``cutoff`` with quality ``q``."""

    def __init__(self, cutoff: float, q: float, sample_rate: float) -> None:
        super().__init__(cutoff, sample_rate, q, BiquadKind.NOTCH)

    @staticmethod
    def get_q(center_freq: float, cutoff_freq: float) -> float:
        """Q from center frequency (f0) and lower cutoff (f1); ``notchFilterGetQ`` in filter.c."""
        if center_freq > 0 and cutoff_freq > 0 and cutoff_freq < center_freq:
            return (
                center_freq
                * cutoff_freq
                / (center_freq * center_freq - cutoff_freq * cutoff_freq)
            )
        return 0.0

    def update_notch(self, cutoff: float, q: float, sample_rate: float) -> None:
        self.update_sos(cutoff, sample_rate, q, BiquadKind.NOTCH)


# --- State-variable filter (SVF, Cytomic trapezoidal / TPT) ---

# Second-order multimode SVF with trapezoidal integration (bilinear-transform match),
# as in Andrew Simper (Cytomic) technical papers, e.g. ``SvfLinearTrapOptimised2.pdf``.
# States ``ic1eq`` / ``ic2eq`` are trapezoidal integrator equivalents; per step
# ``v1`` / ``v2`` are the first- and second-integrator outputs used for mixing.


class SVFFilter(BaseFilter):
    """Abstract TPT SVF: shared ``update`` / ``update_svf`` / ``_step``; subclasses set output mix (Cytomic)."""

    def __init__(self, cutoff: float, sample_rate: float, q: float) -> None:
        self._ic1eq = 0.0
        self._ic2eq = 0.0
        self._q = 1.0
        self._g = 0.0
        self._k = 0.0
        self._a1 = 0.0
        self._a2 = 0.0
        self._a3 = 0.0
        self._valid = False
        self._y_out = 0.0
        self.update_svf(cutoff, q, sample_rate)

    def init(self, cutoff: float, sample_rate: float, q: float) -> None:
        self._ic1eq = self._ic2eq = 0.0
        self.update_svf(cutoff, q, sample_rate)

    def update(self, cutoff: float, sample_rate: float) -> None:
        self.update_svf(cutoff, self._q, sample_rate)

    def update_svf(self, cutoff: float, q: float, sample_rate: float) -> None:
        if cutoff > 0 and sample_rate > 0 and q > 0:
            fc = BaseFilter.limit_cutoff(cutoff, sample_rate)
            g = math.tan(M_PI * fc / sample_rate)
            k = 1.0 / q
            a1 = 1.0 / (1.0 + g * (g + k))
            a2 = g * a1
            a3 = g * a2
            self._g = g
            self._k = k
            self._a1 = a1
            self._a2 = a2
            self._a3 = a3
            self._q = q
            self._valid = True
        else:
            self._q = 1.0
            self._g = self._k = self._a2 = self._a3 = 0.0
            self._a1 = 1.0
            self._valid = False

    def _step(self, v0: float) -> tuple[float, float, float]:
        """One TPT SVF step; returns ``(v0, v1, v2)`` for ``m0*v0 + m1*v1 + m2*v2``."""
        if not self._valid:
            return (v0, 0.0, v0)
        ic1 = self._ic1eq
        ic2 = self._ic2eq
        v3 = v0 - ic2
        v1 = self._a1 * ic1 + self._a2 * v3
        v2 = ic2 + self._a2 * ic1 + self._a3 * v3
        self._ic1eq = 2.0 * v1 - ic1
        self._ic2eq = 2.0 * v2 - ic2
        return (v0, v1, v2)

    @abstractmethod
    def apply(self, x: float) -> float:
        """Run one SVF step and return the selected response."""

    @property
    def output(self) -> float:
        return self._y_out


class SVFLPF(SVFFilter):
    """TPT SVF lowpass: ``m0=0, m1=0, m2=1`` → output ``v2``."""

    def apply(self, x: float) -> float:
        _, _, v2 = self._step(x)
        self._y_out = v2
        return v2


class SVFHPF(SVFFilter):
    """TPT SVF highpass: ``m0=1, m1=-k, m2=-1`` → output ``v0 - k*v1 - v2``."""

    def apply(self, x: float) -> float:
        if not self._valid:
            self._y_out = x
            return x
        v0, v1, v2 = self._step(x)
        y = v0 - self._k * v1 - v2
        self._y_out = y
        return y


class SVFBPF(SVFFilter):
    """TPT SVF bandpass: ``m0=0, m1=k, m2=0`` (Cytomic) → output ``k*v1``."""

    def apply(self, x: float) -> float:
        if not self._valid:
            self._y_out = x
            return x
        _, v1, _ = self._step(x)
        y = self._k * v1
        self._y_out = y
        return y


class SVFAPF(SVFFilter):
    """TPT SVF allpass: ``m0=1, m1=-2*k, m2=0`` → output ``v0 - 2*k*v1``."""

    def apply(self, x: float) -> float:
        v0, v1, _ = self._step(x)
        y = v0 - 2.0 * self._k * v1
        self._y_out = y
        return y


class SVFNotch(SVFFilter):
    """TPT SVF notch: ``m0=1, m1=-k, m2=0`` → output ``v0 - k*v1``."""

    def apply(self, x: float) -> float:
        v0, v1, _ = self._step(x)
        y = v0 - self._k * v1
        self._y_out = y
        return y



if __name__ == "__main__":
    pass
