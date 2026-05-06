#!/usr/bin/env python3

import argparse
import math

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

K_MIN = 0.0
K_MAX = 100.0


class config():
    blade_grip_arm = 10.0
    blade_grip_link = 9.0
    slider_arm = 9.0
    pivot_slider_arm = 18.0
    pivot_slider_zero = 0.0
    pivot_rod_arm = 23.0
    pivot_angle = 0.0
    servo_arm = 14.0


def clamp(value, min_value, max_value):
    """Return value limited to [min_value, max_value]. Requires min_value <= max_value."""
    return min(max(value, min_value), max_value)


def thrust_to_blade_command(u, k):
    safe_k = clamp(k, K_MIN, K_MAX) / 200.0
    sqrt_u = math.copysign(math.sqrt(abs(u)), u)
    return (u * (1 - safe_k) + sqrt_u * safe_k)


def rod_deflection_to_servo_angle(deflection):
    ratio = clamp(deflection / config.servo_arm, -1.0, 1.0)
    return math.asin(ratio)

def pivot_angle_to_rod_deflection(angle):
    dx = math.sin(config.pivot_angle) * config.pivot_rod_arm
    gamma = angle + config.pivot_angle
    rx = math.sin(gamma) * config.pivot_rod_arm
    return rx - dx

def slider_to_pivot_angle(slider):
    sx = slider + config.pivot_slider_zero
    return math.atan2(sx, config.pivot_slider_arm)

def blade_angle_to_slider(alpha):
    # Zero position
    ly_zero = config.blade_grip_arm - config.slider_arm
    lr_zero = config.blade_grip_link
    lx_zero = math.sqrt(max(0.0, lr_zero ** 2 - ly_zero ** 2))

    # Blade position
    ax = math.sin(alpha) * config.blade_grip_arm
    ay = math.cos(alpha) * config.blade_grip_arm
    ly = ay - config.slider_arm
    lr = config.blade_grip_link
    lx = math.sqrt(max(0.0, lr ** 2 - ly ** 2))
    return ax + lx - lx_zero


def blade_angle_to_servo_angle(alpha):
    slider = blade_angle_to_slider(alpha)
    pivot = slider_to_pivot_angle(slider)
    rod_deflection = pivot_angle_to_rod_deflection(pivot)
    return rod_deflection_to_servo_angle(rod_deflection)


def tangent_slope_servo_per_blade_at_origin(k, blade_max_rad):
    """
    Slope d(servo)/d(blade) at the origin for the composed path u -> blade -> servo
    (radians per radian; same as deg/deg on the plot).
    """
    du = 1e-7
    b_m = thrust_to_blade_command(-du, k) * blade_max_rad
    b_p = thrust_to_blade_command(du, k) * blade_max_rad
    s_m = blade_angle_to_servo_angle(b_m)
    s_p = blade_angle_to_servo_angle(b_p)
    db = b_p - b_m
    ds = s_p - s_m
    if abs(db) < 1e-18:
        h = 1e-10
        return (blade_angle_to_servo_angle(h) - blade_angle_to_servo_angle(-h)) / (2.0 * h)
    return ds / db


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot servo angle from requested normalized thrust."
    )
    parser.add_argument("--k", type=float, default=1.0, help=f"Coefficient K. Range: {K_MIN:g}..{K_MAX:g}.")
    parser.add_argument(
        "--blade-max-deg",
        type=float,
        default=45.0,
        help="Maximum blade angle (deg) when |x|=1. Default: 45",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=801,
        help="Samples across thrust range [-1, 1]. Default: 801",
    )
    parser.add_argument("--output", type=str, default=None, help="Optional image output path.")
    return parser.parse_args()


def main():
    args = parse_args()

    if args.k < K_MIN or args.k > K_MAX:
        raise ValueError(f"k must be in [{K_MIN:g}, {K_MAX:g}].")
    if args.blade_max_deg <= 0.0:
        raise ValueError("blade-max-deg must be > 0.")
    if args.samples < 5:
        raise ValueError("samples must be >= 5.")

    thrust_values = []
    servo_deg_values = []
    blade_deg_values = []
    blade_max_rad = math.radians(args.blade_max_deg)

    for i in range(args.samples):
        u = -1.0 + 2.0 * i / (args.samples - 1)
        x_blade = thrust_to_blade_command(u, args.k)
        blade_angle_rad = x_blade * blade_max_rad
        servo_angle_deg = math.degrees(blade_angle_to_servo_angle(blade_angle_rad))

        thrust_values.append(u)
        blade_deg_values.append(math.degrees(blade_angle_rad))
        servo_deg_values.append(servo_angle_deg)

    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(22.0, 10.0))
    fig.subplots_adjust(bottom=0.14)

    blade_line, = ax0.plot(thrust_values, blade_deg_values, linewidth=2.2, label="Blade angle (deg)")
    servo_line, = ax0.plot(thrust_values, servo_deg_values, linewidth=2.4, label="Servo angle (deg)")
    ax0.axhline(0.0, color="k", linestyle=":", linewidth=1.2)
    ax0.axvline(0.0, color="k", linestyle=":", linewidth=1.0, alpha=0.6)
    ax0.grid(True, alpha=0.3)
    ax0.set_xlabel("Requested normalized thrust u")
    ax0.set_ylabel("Angle (deg)")
    ax0.set_ylim(-50.0, 50.0)
    ax0.set_title("Blade and servo angles from requested tail thrust")
    ax0.legend()

    blade_servo_line, = ax1.plot(
        blade_deg_values,
        servo_deg_values,
        linewidth=2.2,
        color="C2",
        zorder=3,
        label="Path as u spans [-1, 1]",
    )
    tangent_m = tangent_slope_servo_per_blade_at_origin(args.k, blade_max_rad)
    x_lin_lo, x_lin_hi = -50.0, 50.0
    tang_line, = ax1.plot(
        [x_lin_lo, x_lin_hi],
        [tangent_m * x_lin_lo, tangent_m * x_lin_hi],
        color="gray",
        linewidth=1.5,
        linestyle="-",
        zorder=2,
        alpha=0.85,
        label=f"Linear tangent at origin (slope≈{tangent_m:.3f})",
    )
    ax1.axhline(0.0, color="k", linestyle=":", linewidth=1.2)
    ax1.axvline(0.0, color="k", linestyle=":", linewidth=1.0, alpha=0.6)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlabel("Blade angle (deg)")
    ax1.set_ylabel("Servo angle (deg)")
    ax1.set_xlim(-50.0, 50.0)
    ax1.set_ylim(-50.0, 50.0)
    ax1.set_aspect("equal", adjustable="box")
    ax1.set_title("Linkage: blade angle to servo angle")
    ax1.legend()

    title = fig.suptitle(f"K={args.k:.3f}, blade_max=+/-{args.blade_max_deg:.1f} deg", fontsize=12)

    slider_ax = fig.add_axes([0.20, 0.05, 0.60, 0.03])
    k_slider = Slider(
        slider_ax,
        "K",
        K_MIN,
        K_MAX,
        valinit=clamp(args.k, K_MIN, K_MAX),
        valstep=1,
    )

    def update(_):
        k = k_slider.val
        blade_cmd_new = [thrust_to_blade_command(u, k) for u in thrust_values]
        blade_deg_new = [math.degrees(x * blade_max_rad) for x in blade_cmd_new]
        servo_deg_new = [
            math.degrees(blade_angle_to_servo_angle(x * blade_max_rad))
            for x in blade_cmd_new
        ]

        blade_line.set_ydata(blade_deg_new)
        servo_line.set_ydata(servo_deg_new)
        blade_servo_line.set_xdata(blade_deg_new)
        blade_servo_line.set_ydata(servo_deg_new)
        m_new = tangent_slope_servo_per_blade_at_origin(k, blade_max_rad)
        tang_line.set_data(
            [x_lin_lo, x_lin_hi],
            [m_new * x_lin_lo, m_new * x_lin_hi],
        )
        tang_line.set_label(f"Linear tangent at origin (slope≈{m_new:.3f})")
        ax1.legend()
        title.set_text(f"K={k:.3f}, blade_max=+/-{args.blade_max_deg:.1f} deg")
        fig.canvas.draw_idle()

    k_slider.on_changed(update)

    if args.output:
        fig.savefig(args.output, dpi=160)
        print(f"Saved: {args.output}")

    plt.show()


if __name__ == "__main__":
    main()
