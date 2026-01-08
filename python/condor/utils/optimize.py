"""Airfoil optimization module using CST parameterization and aerodynamic analysis.

This module provides tools for optimizing airfoil shapes to maximize aerodynamic
efficiency (L/D ratio) while meeting specified design constraints. It uses the
Class-Shape Transformation (CST) method for upper surface parameterization and
elliptical curves for lower surface definition.
"""

import numpy as np
from scipy.optimize import minimize
import random
from .generafoil import elliptique_rounded, curve_from_cst, a_b_from_neutral_point
import matplotlib.pyplot as plt
import aerosandbox as asb


def objective_function(argu, reynolds_number: float, target_neutral_point: float, flat_length: float) -> float:
    """Calculate negative mean L/D ratio for optimization (minimization).
    
    Args:
        argu: CST coefficients array for upper surface
        reynolds_number: Reynolds number for flow analysis
        target_neutral_point: Target neutral angle in degrees
        flat_length: Flat length ratio for lower surface
        
    Returns:
        Negative weighted mean of L/D ratio (for minimization)
    """
    a, b = a_b_from_neutral_point(target_neutral_point, flat_length)
    cst_coefficients = argu
    x = np.linspace(0, 1, 100)

    y_up = curve_from_cst(x, cst_coefficients)
    y_low, alpha_neutral = elliptique_rounded(x, a, b)

    upper_coords = np.array([x[::-1], y_up[::-1]]).T
    lower_coords = np.array([x[1:], y_low[1:]]).T
    coord = np.concatenate([upper_coords, lower_coords], axis=0)

    airfoil = asb.Airfoil("Experimental", coordinates=np.array(coord)).repanel(n_points_per_side=100)
    alpha_test = np.linspace(alpha_neutral - 2, alpha_neutral + 2, 20)

    aero = airfoil.get_aero_from_neuralfoil(
        alpha=alpha_test,
        Re=reynolds_number,
        mach=0,
        include_360_deg_effects=True
    )

    alpha_weights = np.exp(-abs(target_neutral_point - alpha_neutral))
    conf_weights = np.exp(aero['analysis_confidence'])
    total = -np.mean(aero["CL"] / aero["CD"] * alpha_weights * conf_weights)

    return total


def converence_contrain(target_neutral_point, flat_length, reynolds_number=5e5, nb_test=20, nb_cst_coeff=6):
    """Optimize airfoil shape using constrained SLSQP optimization.
    
    Runs multiple optimization attempts with random initial guesses to find
    the best airfoil configuration that maximizes L/D ratio.
    
    Args:
        target_neutral_point: Target neutral angle in degrees
        flat_length: Flat length ratio for lower surface (0 to 1)
        reynolds_number: Reynolds number for aerodynamic analysis (default: 5e5)
        nb_test: Number of optimization runs with different initial guesses (default: 20)
        nb_cst_coeff: Number of CST coefficients for upper surface (default: 6)
        
    Returns:
        List containing [a, b, A0, A1, A2, ...] where a, b define lower surface
        and A0, A1, A2... are optimized CST coefficients for upper surface
    """
    opti_value = []
    opti_coord = []

    for i in range(nb_test):
        INITIAL_A = np.array([random.uniform(0, 0.3) for i in range(nb_cst_coeff)])
        bounds = [(0, 0.5)] * len(INITIAL_A)

        print(f"Optimization test {i+1}/{nb_test} with initial guess: {INITIAL_A}")

        opt_result = minimize(
            fun=objective_function, 
            x0=INITIAL_A, 
            args=(reynolds_number, target_neutral_point, flat_length), 
            method='SLSQP',
            bounds=bounds,
            options={'disp': True, 'maxiter': 50}
        )

        opti_value.append(opt_result['fun'])
        opti_coord.append(opt_result['x'])

    best = np.min(opti_value)
    best_idx = opti_value.index(best)

    a, b = a_b_from_neutral_point(target_neutral_point, flat_length)
    result = [a, b] + list(opti_coord[best_idx])

    return result


def perf_from_coord(y_low, y_up, x, reynolds_number, alpha_target=None):
    """Calculate aerodynamic performance from airfoil coordinates.
    
    Args:
        y_low: Lower surface y-coordinates
        y_up: Upper surface y-coordinates
        x: Chordwise x-coordinates (0 to 1)
        reynolds_number: Reynolds number for analysis
        alpha_target: Angle of attack in degrees (default: 0)
        
    Returns:
        Dictionary containing aerodynamic coefficients (CL, CD, CM)
    """
    if alpha_target is None:
        alpha_target = 0

    upper_coords = np.array([x[::-1], y_up[::-1]]).T
    lower_coords = np.array([x[1:], y_low[1:]]).T
    coord = np.concatenate([upper_coords, lower_coords], axis=0)

    speed = 20  # m/s
    reynolds_number = speed * 0.4 / (1.56e-5)
    mach_number = speed / 340

    airfoil = asb.Airfoil("Experimental", coordinates=np.array(coord)).repanel(n_points_per_side=100)

    aero = airfoil.get_aero_from_neuralfoil(
        alpha=alpha_target,
        Re=reynolds_number,
        mach=mach_number,
        include_360_deg_effects=True
    )

    return aero


def plot_perf_from_argu(argu_coord, reynolds_number):
    """Plot aerodynamic efficiency (L/D) vs angle of attack with confidence overlay.
    
    Args:
        argu_coord: List [a, b, A0, A1, A2, ...] defining airfoil parameters
        reynolds_number: Reynolds number for analysis
    """
    x = np.linspace(0, 1, 100)

    a = (1 - argu_coord[0]) / (2 - argu_coord[0])
    b = argu_coord[1]
    cst_coefficients = argu_coord[2:]

    y_up = curve_from_cst(x, cst_coefficients)
    y_low, alpha_neutral = elliptique_rounded(x, a, b)

    upper_coords = np.array([x[::-1], y_up[::-1]]).T
    lower_coords = np.array([x[1:], y_low[1:]]).T
    coord = np.concatenate([upper_coords, lower_coords], axis=0)

    airfoil = asb.Airfoil("Experimental", coordinates=np.array(coord)).repanel(n_points_per_side=100)

    alpha_range = np.linspace(-5, 15, 200)

    aero = airfoil.get_aero_from_neuralfoil(
        alpha=alpha_range,
        Re=reynolds_number,
        mach=0,
        include_360_deg_effects=True
    )

    plt.scatter(
        np.linspace(-5, 15, 200),
        aero['CL'] / aero['CD'],
        c=aero['analysis_confidence'], 
        cmap='Spectral',               
        label='Données Aérodynamiques',
        marker='+'
    )

    plt.axvline(x=alpha_neutral, color='grey', linestyle='--', label='Alpha Neutre')
    plt.colorbar(label="Analysis Confidence")
    plt.grid()
    plt.xlabel("Angle d'attaque (°)")
    plt.ylabel("Efficacité aérodynamique (CL/CD)")
    plt.show()


def perf_from_argu(argu, reynolds_number, alpha_target=None):
    """Calculate aerodynamic performance from parametric airfoil definition.
    
    Args:
        argu: List [a, b, A0, A1, A2, ...] defining airfoil parameters
        reynolds_number: Reynolds number for analysis
        alpha_target: Relative angle of attack in degrees (default: 0)
        
    Returns:
        Tuple of (aero_dict, alpha_neutral) where aero_dict contains CL, CD, CM
        and alpha_neutral is the neutral angle in degrees
    """
    x = np.linspace(0, 1, 100)

    if alpha_target is None:
        alpha_target = 0

    a = argu[0]
    b = argu[1]
    cst_coefficients = argu[2:]

    y_up = curve_from_cst(x, cst_coefficients)
    y_low, alpha_neutral = elliptique_rounded(x, a, b)

    upper_coords = np.array([x[::-1], y_up[::-1]]).T
    lower_coords = np.array([x[1:], y_low[1:]]).T
    coord = np.concatenate([upper_coords, lower_coords], axis=0)

    airfoil = asb.Airfoil("Experimental", coordinates=np.array(coord)).repanel(n_points_per_side=100)

    aero = airfoil.get_aero_from_neuralfoil(
        alpha=alpha_target + alpha_neutral,
        Re=reynolds_number,
        mach=0,
        include_360_deg_effects=True
    )

    return aero, alpha_neutral


def get_wing_stats_from_coords(coords):
    """Calculate geometric properties of a trapezoidal wing section.
    
    Args:
        coords: List of 4 tuples [(x0,y0), (x1,y1), (x2,y2), (x3,y3)] where
                P0: Root leading edge, P1: Tip leading edge
                P2: Tip trailing edge, P3: Root trailing edge
                
    Returns:
        Dictionary with keys: area, mac_length, lift_center (tuple)
    """
    p0, p1, p2, p3 = coords
    
    span = abs(p1[0] - p0[0])
    c_root = abs(p3[1] - p0[1])
    c_tip = abs(p2[1] - p1[1])
    area = span * (c_root + c_tip) / 2
    
    taper_ratio = c_tip / c_root
    mac = c_root * (2/3) * ((1 + taper_ratio + taper_ratio**2) / (1 + taper_ratio))
    
    x_ac = p0[0] + (span / 3) * ((1 + 2 * taper_ratio) / (1 + taper_ratio))
    y_le_at_xac = p0[1] + (x_ac - p0[0]) * (p1[1] - p0[1]) / (p1[0] - p0[0])
    
    direction = 1 if p3[1] > p0[1] else -1
    y_ac = y_le_at_xac + (direction * 0.25 * mac)
    
    return {
        "area": area,
        "mac_length": mac,
        "lift_center": (x_ac, y_ac)
    }


def lift_data(root_chord, wingspan, sweepback, tapering, profiles, speed, alpha=None, plot_wing=True):
    """Calculate aerodynamic data and center of pressure for a tapered swept wing.
    
    Analyzes a wing divided into multiple sections, each with its own airfoil profile.
    Computes local and total lift, drag, and moment characteristics.
    
    Args:
        root_chord: Root chord length in meters
        wingspan: Total wingspan in meters
        sweepback: Sweepback angle in degrees (positive for aft sweep)
        tapering: Taper ratio (tip_chord / root_chord)
        profiles: List of dicts, each with keys 'params' and 'x_root', ordered tip to root
                  params: [a, b, A0, A1, ...] defining airfoil shape
                  x_root: Spanwise position of section root in meters
        speed: Flight speed in m/s
        alpha: Angle of attack in degrees (default: None uses neutral angle)
        plot_wing: If True, displays wing planform visualization (default: True)
        
    Returns:
        Tuple of (section_data, cg_position) where:
        - section_data: List of dicts with aerodynamic data per section
        - cg_position: Tuple (x, y) of overall center of pressure in meters
    """
    section_data = []
    x = np.linspace(0, 1, 100)
    sweepback_angle = -np.deg2rad(sweepback)

    def get_coordinates_from_surface(x_root, wingspan, ws_length, sweepback, tapering):
        """Generate corner coordinates for a wing section."""
        return [
            (x_root, x_root * np.tan(sweepback)),
            (x_root + ws_length, (x_root + ws_length) * np.tan(sweepback)),
            (x_root + ws_length, (x_root + ws_length) * np.tan(sweepback) - root_chord * (1 - tapering * (x_root + ws_length) / wingspan)),
            (x_root, x_root * np.tan(sweepback) - root_chord * (1 - tapering * x_root / wingspan)),
        ]

    for i in range(len(profiles)):
        airfoil_data = profiles[i]

        if i == len(profiles) - 1:
            ws_length = wingspan - airfoil_data['x_root']
        else:
            ws_length = profiles[i+1]['x_root'] - airfoil_data['x_root']

        coordinates_surface = get_coordinates_from_surface(
            airfoil_data['x_root'], wingspan, ws_length, sweepback_angle, tapering
        )

        result = get_wing_stats_from_coords(coordinates_surface)
        reynolds_number = speed * result['mac_length'] / (1.56e-5)
        aero, alpha_neutral = perf_from_argu(airfoil_data['params'], reynolds_number, alpha)

        local_data = {
            "cl": aero['CL'][0],
            "cd": aero['CD'][0],
            "cm": aero['CM'][0],
            "alpha_neutral": alpha_neutral,
            "surface": result['area'],
            "lift_center": result['lift_center'],
            "mac_length": result['mac_length'],
            "lift": aero['CL'][0] * speed**2 * result['area'] * 0.5 * 1.225,
            "drag": aero['CD'][0] * speed**2 * result['area'] * 0.5 * 1.225, 
            "moment": aero['CM'][0] * speed**2 * result['area'] * 0.5 * 1.225 * result['mac_length'],
        }

        section_data.append(local_data)

        if plot_wing:
            xs = [point[0] for point in coordinates_surface] + [coordinates_surface[0][0]]
            ys = [point[1] for point in coordinates_surface] + [coordinates_surface[0][1]]
            plt.plot(xs, ys, 'b-')
            plt.fill(xs, ys, color='blue', alpha=0.3)
            plt.plot(result['lift_center'][0], result['lift_center'][1], 'ro', markersize=5)
            plt.xlabel('x (m)')
            plt.ylabel('y (m)')
            plt.title('Wing Planform')
            plt.axis('equal')
            plt.grid(True)

    total_lift = sum([section['lift'] for section in section_data])
    cg_x = sum([section['lift_center'][0] * section['lift'] for section in section_data]) / total_lift
    cg_y = sum([section['lift_center'][1] * section['lift'] for section in section_data]) / total_lift
    
    if plot_wing:
        plt.plot(cg_x, cg_y, 'go', markersize=8, label='Wing Lift Center of Gravity')
        plt.legend()
        plt.show()

    return section_data, (cg_x, cg_y)