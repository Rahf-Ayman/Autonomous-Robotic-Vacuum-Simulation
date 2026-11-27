"""Configuration constants and math helpers for the Roomba digital-twin."""

# Motor & battery
MAX_SPEED = 6.28
BATTERY_MAX = 10_000.0
BATTERY_LOW_PCT = 0.20  # return to charger below 20%
ROAMING_DURATION = 1200.0  # max 20 minutes per cleaning phase

# LiDAR sector index ranges (256 rays, 180° FOV)
L0, L1 = 0, 64
FL0, FL1 = 64, 112
F0, F1 = 112, 144
FR0, FR1 = 144, 192
R0, R1 = 192, 256

OBSTACLE_DIST = 0.25
FAR_OBSTACLE_DIST = 0.20
SLOW_DIST = 0.55

# Carpet sensor thresholds
CARPET_FULL = 675
CARPET_EDGE = 655
SURFACE_HEIGHT_DELTA = 0.004

# Docking geometry
CHARGER_X = 2.00
CHARGER_Y = 2.85
PREDOCK_X = 2.00
PREDOCK_Y = 1.80
DOCK_ARM_Y = 2.55
DOCK_CONFIRM_Y = 2.70
DOCK_STALL_RADIUS_Y = 2.45

# Timeouts
ALIGNING_THRESHOLD = 1.5
ALIGNING_TIMEOUT = 10.0
DOCKING_TIMEOUT = 45.0
CHARGING_MAX_TIME = 90.0

# Boustrophedon (zigzag) navigation bounds and parameters
BOUS_Y_MIN = -2.05
BOUS_Y_MAX = 2.15
BOUS_X_MIN = -2.65
BOUS_X_MAX = 2.65
BOUS_STRIP_STEP = 0.35
BOUS_WP_RADIUS = 0.22

WP_FRUSTRATION_TIME = 10.0

SPEED_RAMP = 5.0
LOOK_AHEAD_DIST = 0.55

STUCK_TIMEOUT = 10.0
STUCK_DIST = 0.05

WP_NEAR_THRESH = 1.5

SCAN_MARGIN = 0.30  # safety margin relative to detected wall points (m)
CARPET_CELL_SIZE = 0.25  # grid cell size for carpet map


def clamp(value, lo, hi):
    """Constrain ``value`` to the inclusive range ``[lo, hi]``."""
    return max(lo, min(value, hi))


def angle_error(target, current):
    """Shortest signed angle from current to target in [-180, 180]."""
    error = target - current
    while error > 180.0:
        error -= 360.0
    while error < -180.0:
        error += 360.0
    return error