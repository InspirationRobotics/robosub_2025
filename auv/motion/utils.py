"""
Various mathematical functions
"""

import math


def heading_error(heading, target):
    """
    Calculate heading error between the current and target heading

    Args:
        heading (float): Current heading in degrees
        target (float): Target heading in degrees

    Returns:
        float: Heading error in degrees, handling the case where 359 degrees and 0 degrees are close
    """
    error = target - heading
    if abs(error) > 180:
        error = (error + 360) % 360
    return error


def get_norm(x, y):
    """
    Calculate Euclidean norm(distance) of a vectior

    Args:
        x (float): X-component of the vector
        y (float): Y-component of the vector
    
    Returns:
        float: Euclidean norm of the vector
    """
    norm = math.sqrt(x**2 + y**2)
    return norm

def get_distance(v1, v2):
    """
    Calculate the Euclidean distance between two points

    Args:
        v1 (tuple): Coordinates of the first point (x1, y1)
        v2 (tuple): Coordinates of the second point (x2, y2)
    
    Returns:
        float: Euclidean distance between the two points
    """
    dist = math.sqrt((v1[0] - v2[0])**2 + (v1[1] - v2[1])**2)
    return dist

def rotate_vector(x, y, heading):
    """
    Rotate a vector by a given heading in degrees (counter-clockwise).

    Args:
        x (float): X-component of the vector
        y (float): Y-component of the vector
        heading (float): Angle in degrees to rotate the vector by
    
    Returns:
        tuple: Rotated vector components (x_rot, y_rot)
    """
    theta = math.radians(heading)
    x_rot = x * math.cos(theta) - y * math.sin(theta)
    y_rot = x * math.sin(theta) + y * math.cos(theta)
    return x_rot, y_rot

def inv_rotate_vector(x, y, heading):
    """
    Rotate a vector by the *negative* of the given heading (i.e., inverse rotation)
    in the clockwise direction.

    Args:
        x (float): X-component of the vector
        y (float): Y-component of the vector
        heading (float): Angle in degrees to inverse-rotate the vector by
    
    Returns:
        tuple: Rotated vector components (x_rot, y_rot)
    """
    theta = math.radians(heading)
    x_rot = x * math.cos(theta) + y * math.sin(theta)
    y_rot = -x * math.sin(theta) + y * math.cos(theta)
    return x_rot, y_rot

def get_heading_from_coords(x, y):
    """
    Get heading (angle) from given coordinates
    Assumes the Y-axis is North and the X-axis is East
    
    Parameters:
        x (float): X-coordinate
        y (float): Y-coordinate
    
    Returns:
        float: Heading angle in degrees
    """
    return math.degrees(math.atan2(x, y))
