###############################################################################
# Inverse kinetics for meArm
###############################################################################
#
# adapted for Python by Bob Stone from C++ original by Nick Moriarty May 2014
# Original is here: https://github.com/aquila12/me-arm-ik
#
# refactored by ChatGPT Summer 2025
###############################################################################

# ##############################################################################
# 
# The MIT License (MIT)
# 
# Copyright (c) 2014 Nick Moriarty, Bob Stone
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ##############################################################################

from typing import Optional, Tuple
import math

L1 = 80. # Shoulder to elbow length
L2 = 80. # Elbow to wrist length
L3 = 68. # Wrist to hand plus base centre to shoulder (does not include claw fingers)
   
__all__ = ["inverse_kinematics", "forward_kinematics", "distance_3d"]

def cartesian_to_polar(
        x: float, 
        y: float) -> Tuple[float, float]:
    """Convert Cartesian coordinates (x, y) to polar (r, theta)."""
    r = math.hypot(x, y)
    theta = math.atan2(y, x)
    return r, theta

def polar_to_cartesian(
        r: float, 
        theta: float) -> Tuple[float, float]:
    """Convert polar coordinates (r, theta) to Cartesian (x, y)."""
    return r * math.cos(theta), r * math.sin(theta)

def _angle_from_cosine(
        opposite: float, 
        adjacent1: float, 
        adjacent2: float) -> float:
    """
    Compute angle opposite the 'opposite' side in a triangle using the law of cosines.
    """
    denom = 2.0 * adjacent1 * adjacent2
    if denom == 0.0:
        raise ValueError("Degenerate triangle with zero-length sides")
    cos_val = (adjacent1**2 + adjacent2**2 - opposite**2) / denom
    # Clamp to [-1, 1] to avoid numerical errors
    cos_val = max(-1.0, min(1.0, cos_val))
    return math.acos(cos_val)

def inverse_kinematics(
        x: float, 
        y: float, 
        z: float) -> Optional[Tuple[float, float, float]]:
    """
    Solve for joint angles (theta0, theta1, theta2) given end effector position (x, y, z).
    Returns None if no valid solution exists.
    """
    # Base rotation (top-down view)
    r_xy, theta0 = cartesian_to_polar(y, x)
    # Remove wrist length from base-plane distance
    r_xy -= L3

    # Project into arm's plane
    planar_dist, plane_theta = cartesian_to_polar(r_xy, z)

    try:
        # angle at elbow joint
        angle_b = _angle_from_cosine(L2, L1, planar_dist)
        # angle at shoulder joint projection
        angle_c = _angle_from_cosine(planar_dist, L1, L2)
    except ValueError:
        return None

    theta1 = plane_theta + angle_b
    theta2 = theta1 + angle_c - math.pi

    return theta0, theta1, theta2

def forward_kinematics(
    theta0: float, 
    theta1: float, 
    theta2: float) -> Tuple[float, float, float]:
    """
    Compute end effector position (x, y, z) from joint angles (theta0, theta1, theta2).
    """
    # Calculate in-plane projections
    u1, v1 = polar_to_cartesian(L1, theta1)
    u2, v2 = polar_to_cartesian(L2, theta2)
    u = u1 + u2 + L3
    v = v1 + v2

    # Rotate back into XY plane
    x = u * math.sin(theta0)
    y = u * math.cos(theta0)
    z = v
    return x, y, z

def distance_3d(
    p1: Tuple[float, float, float], p2: Tuple[float, float, float]
) -> float:
    """Compute Euclidean distance between 3D points p1 and p2."""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dz = p2[2] - p1[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)
