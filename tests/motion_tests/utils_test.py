import math
import pytest

from auv.motion.utils import (
    heading_error,
    get_norm,
    get_distance,
    rotate_vector,
    inv_rotate_vector,
    get_heading_from_coords
)
# ------------------------
# Test: heading_error
# ------------------------

@pytest.mark.parametrize("heading, target, expected", [
    (0, 90, 90),
    (90, 0, -90),
    (350, 10, 20),
    (10, 350, -20),
    (0, 0, 0),
])
def test_heading_error(heading, target, expected):
    assert heading_error(heading, target) == pytest.approx(expected, abs=1e-5)

# ------------------------
# Test: get_norm
# ------------------------

@pytest.mark.parametrize("x, y, expected", [
    (3, 4, 5),
    (0, 0, 0),
    (1, 1, math.sqrt(2)),
])
def test_get_norm(x, y, expected):
    assert get_norm(x, y) == pytest.approx(expected, abs=1e-5)

# ------------------------
# Test: get_distance
# ------------------------

@pytest.mark.parametrize("v1, v2, expected", [
    ((0, 0), (3, 4), 5),
    ((1, 1), (1, 1), 0),
    ((-1, -1), (2, 3), math.sqrt(34)),
])
def test_get_distance(v1, v2, expected):
    assert get_distance(v1, v2) == pytest.approx(expected, abs=1e-5)

# ------------------------
# Test: rotate_vector + inv_rotate_vector
# ------------------------

@pytest.mark.parametrize("x, y, angle", [
    (1, 0, 90),
    (0, 1, 180),
    (1, 1, 45),
    (1, 1, -45),
])
def test_rotate_and_inverse(x, y, angle):
    x_rot, y_rot = rotate_vector(x, y, angle)
    x_back, y_back = inv_rotate_vector(x_rot, y_rot, angle)
    assert x_back == pytest.approx(x, abs=1e-5)
    assert y_back == pytest.approx(y, abs=1e-5)

# ------------------------
# Test: get_heading_from_coords
# ------------------------

@pytest.mark.parametrize("x, y, expected", [
    (0, 1, 0),       # North
    (1, 0, 90),      # East
    (0, -1, 180),    # South
    (-1, 0, -90),    # West
    (1, 1, 45),
    (-1, 1, -45),
])
def test_get_heading_from_coords(x, y, expected):
    result = get_heading_from_coords(x, y)
    assert result == pytest.approx(expected, abs=1e-5)
