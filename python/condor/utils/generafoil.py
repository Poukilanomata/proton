"""Airfoil generation module using Class-Shape Transformation (CST) method.

This module provides functions for generating airfoil geometries using the CST
parameterization method for the upper surface and elliptical/flat curves for
the lower surface. It includes utilities for computing neutral angles and
visualizing airfoil profiles.
"""

import numpy as np
from scipy.special import comb
import matplotlib.pyplot as plt


def bernstein_poly(i, n, xi):
    """Calculate Bernstein polynomial B_{i,n}(xi) of degree n.
    
    Args:
        i: Coefficient index (0 to n)
        n: Polynomial order (number of coefficients - 1)
        xi: Normalized chordwise position (x/c), range [0, 1]
        
    Returns:
        Value of Bernstein polynomial at xi
    """
    return (comb(n, i) * (xi**i) * ((1 - xi)**(n - i)))


def cst_shape_function(A, xi):
    """Calculate CST shape function S(xi) for coefficient array A.
    
    Args:
        A: Array of shape coefficients [A_0, A_1, ..., A_n]
        xi: Array of normalized chordwise positions (x/c)
        
    Returns:
        Array of shape function values at each xi
    """
    n = len(A) - 1
    S = np.zeros_like(xi)
    for i in range(n + 1):
        S += A[i] * bernstein_poly(i, n, xi)
    return S


def cst_class_function(xi, N1=0.5, N2=1.0):
    """Calculate CST class function C(xi).
    
    Defines leading and trailing edge behavior. Default values (N1=0.5, N2=1.0)
    produce a rounded leading edge and sharp trailing edge typical of airfoils.
    
    Args:
        xi: Normalized chordwise position array (x/c)
        N1: Leading edge exponent (default: 0.5 for rounded)
        N2: Trailing edge exponent (default: 1.0 for sharp)
        
    Returns:
        Array of class function values at each xi
    """
    return (xi**N1) * ((1 - xi)**N2)


def generate_cst_flat_profile(A_u, x_points, N1=0.5, N2=1.0):
    """Generate CST airfoil with flat lower surface (y_l = 0).
    
    Args:
        A_u: CST coefficients for upper surface
        x_points: Array of chordwise positions (0 to 1)
        N1: Leading edge class parameter (default: 0.5)
        N2: Trailing edge class parameter (default: 1.0)
        
    Returns:
        Tuple of (y_upper, y_lower) coordinate arrays
    """
    xi = x_points
    
    C = cst_class_function(xi, N1, N2)
    S_u = cst_shape_function(A_u, xi)
    y_u = C * S_u
    y_l = np.zeros_like(xi)

    return y_u, y_l


def curve_from_cst(x, A, N1=0.5, N2=1.0):
    """Generate curve from CST parameters.
    
    Args:
        x: Array of chordwise positions (0 to 1)
        A: CST coefficient array
        N1: Leading edge class parameter (default: 0.5)
        N2: Trailing edge class parameter (default: 1.0)
        
    Returns:
        Array of y-coordinates defining the curve
    """
    C = cst_class_function(x, N1, N2)
    S_u = cst_shape_function(A, x)
    y = C * S_u
    return y


def a_b_from_neutral_point(target_neutral_angle, flat_length):
    """Calculate ellipse parameters for specified neutral angle and flat length.
    
    Inverse function of elliptique_rounded. Computes ellipse semi-axes (a, b)
    such that the resulting lower surface has the desired neutral angle.
    
    Args:
        target_neutral_angle: Desired neutral angle in degrees
        flat_length: Flat length ratio (0 to 1), defines transition point
        
    Returns:
        Tuple (a, b) where a and b are ellipse semi-axis parameters
    """
    a = (1 - flat_length) / (2 - flat_length)
    
    x_crit = -a / (a - 1)
    slope_at_x_crit = np.tan(np.deg2rad(target_neutral_angle))
    b = slope_at_x_crit * (a * np.sqrt(x_crit * (2*a - x_crit))) / (x_crit - a)

    return a, b


def elliptique_rounded(x, a=0.1, b=0.2):
    """Generate elliptical lower surface with flat trailing edge extension.
    
    Creates a lower surface combining an elliptical curve near the leading edge
    with a flat extension to the trailing edge. The transition occurs at x_crit
    where the ellipse slope matches the flat section.
    
    Args:
        x: Array of chordwise positions (0 to 1)
        a: Ellipse semi-axis parameter controlling flat length
        b: Ellipse semi-axis parameter controlling depth
        
    Returns:
        Tuple of (y_coordinates, neutral_angle_degrees)
    """
    def x_moins_numpy(a):
        return -a / (a - 1)

    y = []

    def f(xi):
        return b * np.sqrt(1 - ((xi - a) ** 2) / (a ** 2))
    
    def f_deriv(xi):
        return -b * (xi - a) / (a * np.sqrt(xi * (2*a - xi)))

    x_crit = x_moins_numpy(a)

    for i in range(len(x)):
        if x[i] < x_crit: 
            y_tmp = -f(x[i])
        else:
            y_tmp = -(f_deriv(x_crit) * (x[i] - x_crit) + f(x_crit))
        y.append(y_tmp)

    y[-1] = 0

    flat_alpha = np.arctan(f_deriv(x_crit))

    return y, -np.rad2deg(flat_alpha)


def calculate_cst_coefficients(points_x, points_y, n_cst=9):
    """Calculate CST coefficients from discrete airfoil points using least squares.
    
    Fits CST parameterization to a set of (x, y) points representing an airfoil
    surface, useful for reverse-engineering existing airfoil geometries.
    
    Args:
        points_x: Array of x-coordinates (normalized, 0 to 1)
        points_y: Array of corresponding y-coordinates
        n_cst: Number of CST coefficients to compute (default: 9)
        
    Returns:
        Array of n_cst CST coefficients
        
    Raises:
        ValueError: If x-coordinates are not normalized between 0 and 1
    """
    points_x = np.asarray(points_x)
    points_y = np.asarray(points_y)

    if not np.all((points_x >= 0) & (points_x <= 1)):
        raise ValueError("Les coordonnées x doivent être normalisées entre 0 et 1.")
    
    N1 = 0.5
    N2 = 1.0

    C = (points_x**N1) * ((1 - points_x)**N2)
    n = n_cst - 1
    A = np.zeros((len(points_x), n_cst))
    
    def binomial_coefficient(n, i):
        return np.comb(n, i)
        
    for i in range(n_cst):
        K = binomial_coefficient(n, i) * (points_x**i) * ((1 - points_x)**(n - i))
        A[:, i] = C * K
        
    coefficients_cst, residuals, rank, singular_values = np.linalg.lstsq(A, points_y, rcond=None)
    
    return coefficients_cst


def plot_from_argu(params):
    """Plot airfoil from parametric definition.
    
    Args:
        params: List [a, b, A0, A1, A2, ...] where a, b define lower surface
                and A0, A1, ... are CST coefficients for upper surface
    """
    a = params[0]
    b = params[1]

    x = np.linspace(0, 1, 100)
    y_up = curve_from_cst(x, params[2:])
    y_low = elliptique_rounded(x, a, b)[0]

    plot_foil(x, y_up, y_low, alpha=0)


def plot_foil(x, y_up, y_low, alpha):
    """Visualize airfoil profile with optional rotation.
    
    Displays upper and lower surfaces with rotation applied to simulate angle
    of attack visualization.
    
    Args:
        x: Array of chordwise positions
        y_up: Array of upper surface y-coordinates
        y_low: Array of lower surface y-coordinates
        alpha: Rotation angle in radians (angle of attack)
    """
    coord = []
    for i in range(len(x)):
        coord.append((x[i], y_up[i]))

    for i in range(len(x)):
        coord.append((x[i], y_low[i]))

    coord_rota = []
    offset_rota = -np.sin(alpha)
    mat_rota = np.array([[np.cos(alpha), -np.sin(alpha)],
                         [np.sin(alpha),  np.cos(alpha)]])
    
    for i in range(len(coord)):
        coord_rota.append(np.dot(mat_rota, np.array(coord[i])) + np.array([0, offset_rota]))

    x_rotated = [coord_rota[i][0] for i in range(len(coord_rota))]
    y_rotated = [coord_rota[i][1] for i in range(len(coord_rota))]

    plt.plot(x_rotated[:len(x)], y_rotated[:len(x)], c="orange", label="Upper surface")
    plt.plot(x_rotated[len(x):], y_rotated[len(x):], c='blue', label="Lower surface")
    plt.plot(np.linspace(-0.1, 1.1, 2), np.zeros(2), c="green", linestyle='--')

    plt.ylim((-0.6, 0.6))
    plt.xlim((-0.1, 1.1))
    plt.grid()
    plt.legend()
    plt.axis('equal')
    plt.show()