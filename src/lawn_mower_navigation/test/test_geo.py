import pytest

from lawn_mower_navigation.geo import to_geodetic, to_local

ORIGIN = (33.0, 44.0)


def test_origin_is_zero():
    assert to_local(*ORIGIN, ORIGIN) == (0.0, 0.0)


def test_axes_point_east_and_north():
    east, north = to_local(ORIGIN[0], ORIGIN[1] + 1e-4, ORIGIN)
    assert east > 0 and north == 0
    east, north = to_local(ORIGIN[0] + 1e-4, ORIGIN[1], ORIGIN)
    assert north > 0 and east == 0


def test_scale_at_this_latitude():
    # One arc second of latitude is close to 30.8 m, of longitude 30.9 m x cos(latitude).
    _, north = to_local(ORIGIN[0] + 1 / 3600, ORIGIN[1], ORIGIN)
    east, _ = to_local(ORIGIN[0], ORIGIN[1] + 1 / 3600, ORIGIN)
    assert north == pytest.approx(30.81, abs=0.02)
    assert east == pytest.approx(25.96, abs=0.02)


def test_round_trip():
    lat, lon = to_geodetic(12.5, -7.25, ORIGIN)
    assert to_local(lat, lon, ORIGIN) == pytest.approx((12.5, -7.25), abs=1e-6)
