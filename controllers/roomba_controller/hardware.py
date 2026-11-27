"""Webots device initialization, sensor fusion, and motor/LED actuation."""

import math

from config import (
    BATTERY_MAX,
    CARPET_CELL_SIZE,
    CARPET_EDGE,
    CARPET_FULL,
    CHARGER_X,
    CHARGER_Y,
    F0,
    F1,
    FL0,
    FL1,
    FR0,
    FR1,
    L0,
    L1,
    MAX_SPEED,
    R0,
    R1,
    SURFACE_HEIGHT_DELTA,
    clamp,
)


class HardwareInterface:
    """Encapsulate Webots sensors, actuators, and low-level sensor fusion."""

    def __init__(self, robot, timestep):
        self.robot = robot
        self.timestep = timestep

        self.battery = BATTERY_MAX
        self.wall_val = 0.0
        self.carpet_val = 0.0
        self.ranges = []
        self.bumper_hit = 0.0
        self.cliff_hit = False
        self.pos = [0.0, 0.0, 0.0]
        self.bearing = 0.0

        self.d_far_left = float("inf")
        self.d_front_left = float("inf")
        self.d_front = float("inf")
        self.d_front_right = float("inf")
        self.d_far_right = float("inf")

        self.floor_z = None
        self.on_carpet = False

        self._setup_devices()

    def _setup_devices(self):
        self.motor_left = self.robot.getDevice("motor_left")
        self.motor_right = self.robot.getDevice("motor_right")
        for m in (self.motor_left, self.motor_right):
            m.setPosition(float("inf"))
            m.setVelocity(0.0)

        self.lidar = self.robot.getDevice("clearview_lidar")
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()

        self.robot.batterySensorEnable(self.timestep)

        self.bumper = self.robot.getDevice("bumper")
        self.bumper.enable(self.timestep)

        self.wall_sensor = self.robot.getDevice("wall_sensor")
        self.wall_sensor.enable(self.timestep)

        self.carpet_sensor = self.robot.getDevice("carpet_sensor")
        self.carpet_sensor.enable(self.timestep)

        self.cliff_sensors = []
        for name in (
            "cliff_sensor_left",
            "cliff_sensor_right",
            "cliff_sensor_front_left",
            "cliff_sensor_front_right",
        ):
            s = self.robot.getDevice(name)
            s.enable(self.timestep)
            self.cliff_sensors.append(s)

        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice("compass")
        self.compass.enable(self.timestep)

        self.status_led = self.robot.getDevice("status_led")
        self.battery_led = self.robot.getDevice("battery_led")

    def set_motor_speeds(self, left, right):
        """Set bounded angular velocities on left and right wheels."""
        self.motor_left.setVelocity(clamp(left, -MAX_SPEED, MAX_SPEED))
        self.motor_right.setVelocity(clamp(right, -MAX_SPEED, MAX_SPEED))

    def set_leds(self, state):
        """LED pattern per state: (status_led, battery_led)."""
        leds = {
            "VACUUMING": (1, 0),
            "MOPPING": (1, 1),
            "ESCAPE": (0, 0),
            "RETURNING": (1, 1),
            "ALIGNING": (1, 0),
            "DOCKING": (1, 0),
            "CHARGING": (0, 0),
            "UNDOCKING": (1, 0),
            "FINISHED": (1, 1),
        }
        s, b = leds.get(state, (1, 0))
        self.status_led.set(s)
        self.battery_led.set(b)

    def _get_bearing(self):
        val = self.compass.getValues()
        if not val or math.isnan(val[0]):
            return 0.0
        deg = math.degrees(math.atan2(val[1], val[0]))
        return deg + 360.0 if deg < 0.0 else deg

    def _lidar_min(self, start, end):
        sector = [r for r in self.ranges[start:end] if 0.02 < r != float("inf")]
        return min(sector) if sector else float("inf")

    def _read_lidar_sectors(self):
        if not self.ranges:
            self.d_far_left = self.d_front_left = self.d_front = self.d_front_right = (
                self.d_far_right
            ) = float("inf")
            return
        self.d_far_left = self._lidar_min(L0, L1)
        self.d_front_left = self._lidar_min(FL0, FL1)
        self.d_front = self._lidar_min(F0, F1)
        self.d_front_right = self._lidar_min(FR0, FR1)
        self.d_far_right = self._lidar_min(R0, R1)

    def read_sensors(self, state, carpet_map):
        """Poll all Webots devices and update fused carpet/cliff/battery states."""
        self.battery = self.robot.batterySensorGetValue()
        self.wall_val = self.wall_sensor.getValue()
        self.carpet_val = self.carpet_sensor.getValue()
        self.ranges = self.lidar.getRangeImage()
        self.bumper_hit = self.bumper.getValue()
        cliff_raw = any(s.getValue() < 100.0 for s in self.cliff_sensors)
        self.pos = self.gps.getValues()

        near_charger = (
            math.hypot(self.pos[0] - CHARGER_X, self.pos[1] - CHARGER_Y) < 0.65
        )
        if self.floor_z is None and state == "VACUUMING" and not near_charger:
            self.floor_z = self.pos[2]

        raised_surface = (
            self.floor_z is not None
            and self.pos[2] > self.floor_z + SURFACE_HEIGHT_DELTA
        )

        cx = int(self.pos[0] / CARPET_CELL_SIZE)
        cy = int(self.pos[1] / CARPET_CELL_SIZE)
        gps_on = bool(carpet_map) and (cx, cy) in carpet_map

        confirmed_sensor = self.carpet_val > CARPET_FULL and (
            raised_surface or gps_on
        )
        self.on_carpet = (raised_surface or confirmed_sensor) and not near_charger
        near_rug = self.on_carpet or gps_on

        # Dark carpet and its raised edge can look like a drop to the cliff sensors.
        self.cliff_hit = (
            cliff_raw and self.carpet_val <= CARPET_EDGE and not near_rug
        )

        self.bearing = self._get_bearing()
        self._read_lidar_sectors()

        pct = int((self.battery / BATTERY_MAX) * 100) if self.battery >= 0 else 0
        if pct <= 20 and state not in ("CHARGING", "DOCKING"):
            self.battery_led.set(1)

        if state == "VACUUMING" and self.on_carpet:
            if self.carpet_val > CARPET_FULL:
                bearing_rad = math.radians(self.bearing)
                sx = self.pos[0] + 0.16 * math.sin(bearing_rad)
                sy = self.pos[1] + 0.16 * math.cos(bearing_rad)
            else:
                sx, sy = self.pos[0], self.pos[1]
            carpet_map.add((int(sx / CARPET_CELL_SIZE), int(sy / CARPET_CELL_SIZE)))

        return pct