"""Latitude and longitude to local east and north metres around an origin.

Uses the radii of curvature of the WGS 84 ellipsoid at the origin. Over a
lawn the error against the exact conversion is far below a millimetre.
"""

import math

WGS84_A = 6378137.0                # m, semi-major axis
WGS84_E2 = 6.69437999014e-3        # first eccentricity squared


def radii(latitude):
    """Meridian and prime-vertical radii of curvature [m] at a latitude [deg]."""
    s2 = math.sin(math.radians(latitude)) ** 2
    root = math.sqrt(1 - WGS84_E2 * s2)
    return WGS84_A * (1 - WGS84_E2) / root ** 3, WGS84_A / root


def to_local(latitude, longitude, origin):
    """(east, north) [m] of a position [deg] from origin = (latitude, longitude) [deg]."""
    meridian, prime = radii(origin[0])
    east = math.radians(longitude - origin[1]) * prime * math.cos(math.radians(origin[0]))
    north = math.radians(latitude - origin[0]) * meridian
    return east, north


def to_geodetic(east, north, origin):
    """Inverse of to_local: (latitude, longitude) [deg]."""
    meridian, prime = radii(origin[0])
    return (origin[0] + math.degrees(north / meridian),
            origin[1] + math.degrees(east / (prime * math.cos(math.radians(origin[0])))))
