#!/usr/bin/env python3

import math
import matplotlib.pyplot as plt
import numpy as np


class config():
    blade_grip_arm       = 10.0
    blade_grip_link      = 9.5
    slider_arm           = 10.0
    pivot_slider_dist    = 17.0
    pivot_slider_zero    = 0.0
    pivot_rod_arm        = 23.0
    pivot_angle          = 0 #math.radians(5.0)
    servo_arm            = 12.0


def rod_deflection_to_servo_angle( deflection: float ) -> float:
    theta = math.asin(deflection / config.servo_arm)
    return theta

def pivot_angle_to_rod_deflection( angle: float ) -> float:
    Dx = math.sin(config.pivot_angle) * config.pivot_rod_arm
    gamma = angle + config.pivot_angle
    Rx = math.sin(gamma) * config.pivot_rod_arm
    return Rx - Dx

def slider_to_pivot_angle( slider: float ) -> float:
    sx = slider + config.pivot_slider_zero
    gamma = math.atan2(sx, config.pivot_slider_dist)
    return gamma

def blade_angle_to_slider( alpha: float ) -> float:
    # Zero position
    Ly = config.blade_grip_arm - config.slider_arm
    Lr = config.blade_grip_link
    Lx = math.sqrt(Lr**2 - Ly**2)
    # Blade position
    ax = math.sin(alpha) * config.blade_grip_arm
    ay = math.cos(alpha) * config.blade_grip_arm
    ly = ay - config.slider_arm
    lr = config.blade_grip_link
    lx = math.sqrt(lr**2 - ly**2)
    d = ax + lx - Lx
    return d

def blade_angle_to_servo_angle( alpha: float ) -> float:
    slider = blade_angle_to_slider(alpha)
    pivot = slider_to_pivot_angle(slider)
    rod_deflection = pivot_angle_to_rod_deflection(pivot)
    servo_angle = rod_deflection_to_servo_angle(rod_deflection)
    return servo_angle



def polynomial_approx_blade_to_servo( degree: int ):
    blade_angles_deg = np.arange(-45.0, 46.0, 1.0)
    blade_angles_rad = np.radians(blade_angles_deg)
    servo_angles_rad = np.array([blade_angle_to_servo_angle(alpha) for alpha in blade_angles_rad])
    coeffs = np.polyfit(blade_angles_rad, servo_angles_rad, degree)
    return coeffs

def evaluate_polynomial(coeffs, blade_angle_rad: float) -> float:
    return float(np.polyval(coeffs, blade_angle_rad))

def print_poly_coeffs_int32(coeffs, degree: int, scale_multiplier: int = 16384):
    coeffs_int32 = np.round(np.array(coeffs) * scale_multiplier).astype(np.int32)
    coeffs_str = ", ".join([str(int(c)) for c in coeffs_int32])
    print(f"// degree={degree}, coefficients in radians, scale_multiplier={scale_multiplier}")
    print(f"static const int32_t blade_to_servo_poly_deg_{degree}[{len(coeffs_int32)}] = {{{coeffs_str}}};")

def plot_sweep():
    angle_degrees = list(range(-45, 46))
    slider_positions = []
    pivot_angles_deg = []
    rod_deflections = []
    servo_angles_deg = []

    for angle_deg in angle_degrees:
        alpha = math.radians(angle_deg)
        slider = blade_angle_to_slider(alpha)
        pivot = slider_to_pivot_angle(slider)
        rod_deflection = pivot_angle_to_rod_deflection(pivot)
        servo_angle = rod_deflection_to_servo_angle(rod_deflection)
        slider_positions.append(slider)
        pivot_angles_deg.append(math.degrees(pivot))
        rod_deflections.append(rod_deflection)
        servo_angles_deg.append(math.degrees(servo_angle))

    plt.figure(figsize=(16, 12), dpi=200)
    plt.plot(angle_degrees, slider_positions, label="Slider distance (mm)")
    plt.plot(angle_degrees, pivot_angles_deg, label="Pivot angle (deg)")
    plt.plot(angle_degrees, rod_deflections, label="Rod deflection (mm)")
    plt.plot(angle_degrees, servo_angles_deg, label="Servo angle (deg)")
    plt.xlabel("Blade angle (deg)")
    plt.ylabel("Output value")
    plt.title("Tail sweep: blade angle to slider and pivot")
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_blade_to_servo_approx():
    blade_angles_deg = np.arange(-45.0, 46.0, 1.0)
    blade_angles_rad = np.radians(blade_angles_deg)
    servo_real_rad = np.array([blade_angle_to_servo_angle(alpha) for alpha in blade_angles_rad])
    servo_real_deg = np.degrees(servo_real_rad)

    fig, (ax_curve, ax_error) = plt.subplots(2, 1, figsize=(16, 12), dpi=200, sharex=True)
    ax_curve.plot(blade_angles_deg, servo_real_deg, label="Real curve")

    for degree in range(2, 7):
        coeffs = polynomial_approx_blade_to_servo(degree=degree)
        poly = np.poly1d(coeffs)
        print(f"Blade angle -> servo angle polynomial in radians (deg={degree}):")
        print(poly)
        print_poly_coeffs_int32(coeffs, degree)
        servo_approx_rad = np.array([evaluate_polynomial(coeffs, angle_rad) for angle_rad in blade_angles_rad])
        servo_approx_deg = np.degrees(servo_approx_rad)
        servo_error_deg = servo_approx_deg - servo_real_deg
        ax_curve.plot(blade_angles_deg, servo_approx_deg, "--", label=f"Polynomial approx (deg={degree})")
        ax_error.plot(blade_angles_deg, servo_error_deg, label=f"Error (deg={degree})")

    ax_curve.set_ylabel("Servo angle (deg)")
    ax_curve.set_title("Blade angle to servo angle: real vs polynomial approx sweep (deg 2..6)")
    ax_curve.legend()
    ax_curve.grid(True)

    ax_error.set_xlabel("Blade angle (deg)")
    ax_error.set_ylabel("Approx error (deg)")
    ax_error.legend()
    ax_error.grid(True)

    fig.tight_layout()
    plt.show()


def main():
    plot_sweep()
    plot_blade_to_servo_approx()

if __name__ == "__main__":
    main()
