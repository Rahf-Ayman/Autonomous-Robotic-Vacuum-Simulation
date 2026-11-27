"""Coverage waypoint generation, carpet grid queries, and cleaning/return steering."""

import math

from config import (
    BOUS_STRIP_STEP,
    BOUS_WP_RADIUS,
    BOUS_X_MAX,
    BOUS_X_MIN,
    BOUS_Y_MAX,
    BOUS_Y_MIN,
    CARPET_CELL_SIZE,
    CARPET_EDGE,
    FAR_OBSTACLE_DIST,
    LOOK_AHEAD_DIST,
    MAX_SPEED,
    OBSTACLE_DIST,
    PREDOCK_X,
    PREDOCK_Y,
    SCAN_MARGIN,
    SLOW_DIST,
    SPEED_RAMP,
    WP_FRUSTRATION_TIME,
    WP_NEAR_THRESH,
    angle_error,
    clamp,
)


class NavigationPlanner:
    """Manage boustrophedon coverage, carpet avoidance, and return trajectories."""

    def __init__(self, hw, dt):
        self.hw = hw
        self.dt = dt

        self.carpet_map = set()
        self.bous_waypoints = self.generate_waypoints()
        self.bous_wp_idx = 0
        self.wp_done = {"VACUUMING": set(), "MOPPING": set()}
        self.wp_skipped_count = 0

        self.wp_prev_dist = float("inf")
        self.wp_progress_time = 0.0

        self.scan_wall_pts = []
        self.room_bounds_computed = False
        self.last_scan_collect_t = 0.0

        self.exploration_factor = 1.0
        self.cleaning_speed = 0.0

        self.in_carpet_escape = False
        self.carpet_escape_count = 0
        self.carpet_escape_target = (0.0, 0.0)

        self.cleaning_via = None
        self.cleaning_vias = []

        self.returning_start_time = 0.0
        self.returning_via = None
        self.returning_vias = []

    # ── waypoint generation ───────────────────────────────────────────────────
    def strip_x_bounds(self, y, x_min, x_max):
        x_west = x_min
        x_east = x_max

        # Keep clear of the office chair and desk.
        if -0.75 <= y <= 0.45:
            x_west = max(x_west, -1.10)

        # Keep clear of the bed, including its corner approaches.
        if -1.00 <= y <= 1.40:
            x_east = min(x_east, 1.70)

        return x_west, x_east

    def generate_boustrophedon(self, x_min, x_max, y_min, y_max):
        pts = []
        y = y_max
        going_east = True
        while y >= y_min - 0.01:
            yr = round(y, 2)
            x_west, x_east = self.strip_x_bounds(yr, x_min, x_max)
            if going_east:
                pts.append((x_west, yr))
                pts.append((x_east, yr))
            else:
                pts.append((x_east, yr))
                pts.append((x_west, yr))
            y -= BOUS_STRIP_STEP
            going_east = not going_east
        return pts

    def generate_waypoints(self):
        return self.generate_boustrophedon(
            BOUS_X_MIN, BOUS_X_MAX, BOUS_Y_MIN, BOUS_Y_MAX
        )

    def generate_mopping_waypoints(self):
        """Waypoints for mopping: full-width strips outside carpet, west-only inside."""
        if self.carpet_map:
            cy_min = self.min_carpet_y()
            cy_max = self.max_carpet_y()
            cx_west = min(
                (cx + 0.5) * CARPET_CELL_SIZE for (cx, _) in self.carpet_map
            )
        else:
            cy_min = cy_max = 0.0
            cx_west = BOUS_X_MIN
        buffer = 0.35

        cy_lo = cy_min - buffer
        cy_hi = cy_max + buffer

        pts = []
        y = BOUS_Y_MAX
        going_east = True

        while y >= BOUS_Y_MIN - 0.01:
            yr = round(y, 2)
            x_west, x_east = self.strip_x_bounds(yr, BOUS_X_MIN, BOUS_X_MAX)

            in_carpet_zone = bool(self.carpet_map) and cy_lo <= yr <= cy_hi
            if in_carpet_zone:
                x_east = min(x_east, cx_west - 0.45)

            if going_east:
                pts.append((x_west, yr))
                pts.append((x_east, yr))
            else:
                pts.append((x_east, yr))
                pts.append((x_west, yr))

            y -= BOUS_STRIP_STEP
            next_y = round(y, 2)
            next_in_carpet_zone = (
                bool(self.carpet_map) and cy_lo <= next_y <= cy_hi
            )
            if in_carpet_zone and not next_in_carpet_zone:
                going_east = True
            else:
                going_east = not going_east

        n_full = sum(
            1
            for (_, yy) in pts[::2]
            if not (bool(self.carpet_map) and cy_lo <= yy <= cy_hi)
        )
        n_west = sum(
            1 for (_, yy) in pts[::2] if bool(self.carpet_map) and cy_lo <= yy <= cy_hi
        )
        print(
            f"[MOPPING] {len(pts)} waypoints: {n_full} full-width + "
            f"{n_west} west-only strips (carpet zone {cy_lo:.2f}..{cy_hi:.2f}m)"
        )
        return pts

    def compute_room_bounds(self, keep_wp_done=False):
        if len(self.scan_wall_pts) < 50:
            print(
                f"[PASSIVE SCAN] Too few points ({len(self.scan_wall_pts)}) "
                f"→ keeping default room dimensions"
            )
            return

        xs = sorted(p[0] for p in self.scan_wall_pts)
        ys = sorted(p[1] for p in self.scan_wall_pts)
        n = len(xs)

        p5 = max(0, n // 20)
        p95 = min(n - 1, n - n // 20)

        new_x_min = xs[p5] + SCAN_MARGIN
        new_x_max = xs[p95] - SCAN_MARGIN
        new_y_min = ys[p5] + SCAN_MARGIN
        new_y_max = ys[p95] - SCAN_MARGIN

        if new_x_max - new_x_min < 1.0 or new_y_max - new_y_min < 1.0:
            print(
                f"[PASSIVE SCAN] Unreasonable room bounds "
                f"X=[{new_x_min:.2f},{new_x_max:.2f}], "
                f"Y=[{new_y_min:.2f},{new_y_max:.2f}] → keeping defaults"
            )
            return

        print(
            f"[SCANNING] Room discovered: "
            f"X=[{new_x_min:.2f}, {new_x_max:.2f}], "
            f"Y=[{new_y_min:.2f}, {new_y_max:.2f}] "
            f"({n} points, margin={SCAN_MARGIN}m)"
        )

        old_count = len(self.bous_waypoints)
        self.bous_waypoints = self.generate_boustrophedon(
            new_x_min, new_x_max, new_y_min, new_y_max
        )
        if not keep_wp_done:
            self.bous_wp_idx = 0
            self.wp_done = {"VACUUMING": set(), "MOPPING": set()}
        print(
            f"[PASSIVE SCAN] Waypoints updated: "
            f"{old_count} → {len(self.bous_waypoints)} WP "
            f"({'wp_done kept' if keep_wp_done else 'fresh start'})"
        )

    # ── carpet helpers ────────────────────────────────────────────────────────
    def on_carpet_gps(self):
        if not self.carpet_map:
            return False
        cx = int(self.hw.pos[0] / CARPET_CELL_SIZE)
        cy = int(self.hw.pos[1] / CARPET_CELL_SIZE)
        return (cx, cy) in self.carpet_map

    def wp_in_carpet_zone(self, wx, wy, buffer=0.20):
        if not self.carpet_map:
            return False
        return (self.min_carpet_y() - buffer) <= wy <= (self.max_carpet_y() + buffer)

    def min_carpet_y(self):
        if not self.carpet_map:
            return 0.0
        return min((cy + 0.5) * CARPET_CELL_SIZE for (_, cy) in self.carpet_map)

    def max_carpet_y(self):
        if not self.carpet_map:
            return 0.0
        return max((cy + 0.5) * CARPET_CELL_SIZE for (_, cy) in self.carpet_map)

    def handle_carpet_escape(self):
        carpet_y_min = self.min_carpet_y()
        gps_on = self.on_carpet_gps()
        gps_near = False
        if self.carpet_map:
            cx_min = min(cx for (cx, _) in self.carpet_map) * CARPET_CELL_SIZE
            cx_max = (max(cx for (cx, _) in self.carpet_map) + 1) * CARPET_CELL_SIZE
            cy_min = self.min_carpet_y() - CARPET_CELL_SIZE
            cy_max = self.max_carpet_y() + CARPET_CELL_SIZE
            gps_near = (
                cx_min - 0.20 <= self.hw.pos[0] <= cx_max + 0.20
                and cy_min - 0.20 <= self.hw.pos[1] <= cy_max + 0.25
            )
        trigger = (
            self.hw.on_carpet
            or gps_on
            or (self.hw.carpet_val > CARPET_EDGE and gps_near)
        )

        if not self.in_carpet_escape and (trigger or gps_on):
            self.in_carpet_escape = True
            self.carpet_escape_count += 1
            self.carpet_escape_target = (self.hw.pos[0], carpet_y_min - 0.50)

        if not self.in_carpet_escape:
            return False

        sensor_clear = (
            self.hw.carpet_val < (CARPET_EDGE - 5) and not self.hw.on_carpet
        )
        if sensor_clear and not gps_on:
            self.in_carpet_escape = False
            return False

        tx, ty = self.carpet_escape_target
        dx = tx - self.hw.pos[0]
        dy = ty - self.hw.pos[1]
        tb = math.degrees(math.atan2(dx, dy))
        if tb < 0.0:
            tb += 360.0
        err = angle_error(tb, self.hw.bearing)
        turn = clamp(0.035 * err, -0.25 * MAX_SPEED, 0.25 * MAX_SPEED)
        drive_speed = 0.45 * MAX_SPEED * clamp(1.0 - abs(err) / 50.0, 0.0, 1.0)
        self.hw.set_motor_speeds(drive_speed + turn, drive_speed - turn)
        return True

    def follow_cleaning_via(self):
        tx, ty = self.cleaning_via
        if math.hypot(tx - self.hw.pos[0], ty - self.hw.pos[1]) < 0.30:
            if self.cleaning_vias:
                self.cleaning_via = self.cleaning_vias.pop(0)
            else:
                self.cleaning_via = None
            return

        target_deg = math.degrees(
            math.atan2(tx - self.hw.pos[0], ty - self.hw.pos[1])
        )
        if target_deg < 0.0:
            target_deg += 360.0
        err = angle_error(target_deg, self.hw.bearing)
        turn = clamp(0.030 * err, -0.25 * MAX_SPEED, 0.25 * MAX_SPEED)
        drive_speed = 0.45 * MAX_SPEED * clamp(1.0 - abs(err) / 50.0, 0.0, 1.0)

        if self.hw.d_front < OBSTACLE_DIST:
            if self.hw.d_far_left >= self.hw.d_far_right:
                self.hw.set_motor_speeds(-0.20 * MAX_SPEED, 0.50 * MAX_SPEED)
            else:
                self.hw.set_motor_speeds(0.50 * MAX_SPEED, -0.20 * MAX_SPEED)
        else:
            self.hw.set_motor_speeds(drive_speed + turn, drive_speed - turn)

    # ── active cleaning navigation ────────────────────────────────────────────
    def step_cleaning(self, t, allow_carpet):
        """Drive one cleaning control step. Returns True if phase completed."""
        phase_key = "VACUUMING" if allow_carpet else "MOPPING"
        if len(self.wp_done[phase_key]) >= len(self.bous_waypoints):
            skip_str = (
                f" | {self.wp_skipped_count} WPs skipped"
                if self.wp_skipped_count
                else ""
            )
            print(
                f"[MISSION] {phase_key} complete "
                f"({len(self.wp_done[phase_key])}/{len(self.bous_waypoints)} WP{skip_str})"
            )
            return True

        if not allow_carpet and self.handle_carpet_escape():
            return False

        if self.cleaning_via is not None:
            self.follow_cleaning_via()
            return False

        obs_front = self.hw.d_front < OBSTACLE_DIST
        obs_front_left = self.hw.d_front_left < OBSTACLE_DIST
        obs_front_right = self.hw.d_front_right < OBSTACLE_DIST
        obs_far_left = self.hw.d_far_left < FAR_OBSTACLE_DIST
        obs_far_right = self.hw.d_far_right < FAR_OBSTACLE_DIST
        slow_front = self.hw.d_front < SLOW_DIST

        any_obstacle = (
            obs_front
            or obs_front_left
            or obs_front_right
            or obs_far_left
            or obs_far_right
        )

        if any_obstacle:
            self.cleaning_speed = max(0.0, self.cleaning_speed - 0.15 * MAX_SPEED)
            self.wp_progress_time = 0.0
            self.wp_prev_dist = float("inf")

        if obs_front:
            if self.hw.d_far_left >= self.hw.d_far_right:
                self.hw.set_motor_speeds(-0.20 * MAX_SPEED, 0.55 * MAX_SPEED)
            else:
                self.hw.set_motor_speeds(0.55 * MAX_SPEED, -0.20 * MAX_SPEED)

        elif obs_front_left and obs_front_right:
            if self.hw.d_front > 0.50:
                self.hw.set_motor_speeds(0.25 * MAX_SPEED, 0.25 * MAX_SPEED)
            else:
                if self.hw.d_far_left >= self.hw.d_far_right:
                    self.hw.set_motor_speeds(-0.20 * MAX_SPEED, 0.55 * MAX_SPEED)
                else:
                    self.hw.set_motor_speeds(0.55 * MAX_SPEED, -0.20 * MAX_SPEED)

        elif obs_front_left and not obs_front_right:
            self.hw.set_motor_speeds(0.58 * MAX_SPEED, 0.12 * MAX_SPEED)

        elif obs_front_right and not obs_front_left:
            self.hw.set_motor_speeds(0.12 * MAX_SPEED, 0.58 * MAX_SPEED)

        elif obs_far_left and not obs_far_right:
            self.hw.set_motor_speeds(0.52 * MAX_SPEED, 0.30 * MAX_SPEED)

        elif obs_far_right and not obs_far_left:
            self.hw.set_motor_speeds(0.30 * MAX_SPEED, 0.52 * MAX_SPEED)

        elif slow_front:
            base = 0.35 * MAX_SPEED
            if self.hw.d_front_left >= self.hw.d_front_right:
                self.hw.set_motor_speeds(base * 0.6, base)
            else:
                self.hw.set_motor_speeds(base, base * 0.6)

        else:
            wp_x, wp_y = self.bous_waypoints[self.bous_wp_idx]
            dx_wp = wp_x - self.hw.pos[0]
            dy_wp = wp_y - self.hw.pos[1]
            dist_wp = math.hypot(dx_wp, dy_wp)

            if dist_wp < WP_NEAR_THRESH:
                if self.wp_progress_time == 0.0:
                    self.wp_progress_time = t
                    self.wp_prev_dist = dist_wp

                if dist_wp < self.wp_prev_dist - 0.04:
                    self.wp_progress_time = t
                    self.wp_prev_dist = dist_wp
                elif (t - self.wp_progress_time) > WP_FRUSTRATION_TIME:
                    self.wp_done[phase_key].add(self.bous_wp_idx)
                    self.bous_wp_idx = (self.bous_wp_idx + 1) % len(self.bous_waypoints)
                    self.wp_skipped_count += 1
                    self.wp_progress_time = t
                    self.wp_prev_dist = float("inf")
                    wp_x, wp_y = self.bous_waypoints[self.bous_wp_idx]
                    dx_wp = wp_x - self.hw.pos[0]
                    dy_wp = wp_y - self.hw.pos[1]
                    dist_wp = math.hypot(dx_wp, dy_wp)
            else:
                self.wp_progress_time = 0.0
                self.wp_prev_dist = dist_wp

            if dist_wp < BOUS_WP_RADIUS:
                self.wp_done[phase_key].add(self.bous_wp_idx)
                self.bous_wp_idx = (self.bous_wp_idx + 1) % len(self.bous_waypoints)
                self.wp_progress_time = t
                self.wp_prev_dist = float("inf")

                if self.bous_wp_idx % 2 == 0 and self.bous_wp_idx != 0:
                    next_wp = self.bous_waypoints[self.bous_wp_idx]
                    strip_n = self.bous_wp_idx // 2
                    total_s = len(self.bous_waypoints) // 2
                    n_done = len(self.wp_done[phase_key])
                    skipped_str = (
                        f" | {self.wp_skipped_count} skipped"
                        if self.wp_skipped_count
                        else ""
                    )
                    print(
                        f"[NAV] Strip {strip_n}/{total_s} → Y={next_wp[1]:.2f} "
                        f"[{n_done}/{len(self.bous_waypoints)} WP{skipped_str}]"
                    )
                wp_x, wp_y = self.bous_waypoints[self.bous_wp_idx]
                dx_wp = wp_x - self.hw.pos[0]
                dy_wp = wp_y - self.hw.pos[1]
                dist_wp = math.hypot(dx_wp, dy_wp)

            target_deg = math.degrees(math.atan2(dx_wp, dy_wp))
            if target_deg < 0.0:
                target_deg += 360.0
            if dist_wp < LOOK_AHEAD_DIST:
                next_idx = (self.bous_wp_idx + 1) % len(self.bous_waypoints)
                nwx, nwy = self.bous_waypoints[next_idx]
                next_deg = math.degrees(
                    math.atan2(nwx - self.hw.pos[0], nwy - self.hw.pos[1])
                )
                if next_deg < 0.0:
                    next_deg += 360.0

                blend = clamp(1.0 - dist_wp / LOOK_AHEAD_DIST, 0.0, 0.50)
                diff = angle_error(next_deg, target_deg)
                target_deg = (target_deg + blend * diff) % 360.0

            bearing_err = angle_error(target_deg, self.hw.bearing)
            turn = clamp(0.030 * bearing_err, -0.25 * MAX_SPEED, 0.25 * MAX_SPEED)

            wall_corr = 0.0
            if self.hw.wall_val > 80.0:
                wall_err = 380.0 - self.hw.wall_val
                wall_corr = clamp(
                    0.0008 * wall_err, -0.06 * MAX_SPEED, 0.06 * MAX_SPEED
                )

            total_turn = clamp(turn + wall_corr, -0.25 * MAX_SPEED, 0.25 * MAX_SPEED)

            if self.exploration_factor < 1.0:
                self.exploration_factor = min(
                    1.0,
                    self.exploration_factor
                    + (1.0 - self.exploration_factor) * 0.025 * self.dt,
                )
            target_speed = 0.55 * MAX_SPEED * self.exploration_factor
            ramp_delta = SPEED_RAMP * self.dt
            self.cleaning_speed = clamp(
                self.cleaning_speed
                + clamp(target_speed - self.cleaning_speed, -ramp_delta, ramp_delta),
                0.0,
                MAX_SPEED,
            )

            proximity_speed = (
                1.0 if allow_carpet else clamp(dist_wp / 0.60, 0.15, 1.0)
            )
            drive_speed = (
                self.cleaning_speed
                * proximity_speed
                * clamp(1.0 - abs(bearing_err) / 50.0, 0.0, 1.0)
            )
            self.hw.set_motor_speeds(
                drive_speed + total_turn, drive_speed - total_turn
            )

            # Passive wall mapping (VACUUMING only, max 1×/5s)
            if (
                allow_carpet
                and not self.room_bounds_computed
                and t - self.last_scan_collect_t >= 5.0
                and self.hw.ranges
            ):
                self.last_scan_collect_t = t
                bearing_rad = math.radians(self.hw.bearing)
                sensor_x = self.hw.pos[0] + 0.15 * math.sin(bearing_rad)
                sensor_y = self.hw.pos[1] + 0.15 * math.cos(bearing_rad)
                n_rays = len(self.hw.ranges)
                for i, r in enumerate(self.hw.ranges):
                    if 0.10 < r < 5.0:
                        offset_deg = -90.0 + (180.0 / max(n_rays - 1, 1)) * i
                        wb_rad = bearing_rad + math.radians(offset_deg)
                        self.scan_wall_pts.append(
                            (
                                sensor_x + r * math.sin(wb_rad),
                                sensor_y + r * math.cos(wb_rad),
                            )
                        )

        return False

    # ── return-to-dock navigation ─────────────────────────────────────────────
    def step_returning(self, t):
        """Drive toward pre-dock point. Returns True when ready to align."""
        if self.returning_start_time == 0.0:
            self.returning_start_time = t
            self.returning_vias.clear()
            if self.hw.pos[1] < 0.30:
                self.returning_vias = [(-1.10, -1.00), (-1.10, 0.75)]
                self.returning_via = self.returning_vias.pop(0)
            else:
                self.returning_via = None

        if self.returning_via is not None:
            tx, ty = self.returning_via
            if math.hypot(tx - self.hw.pos[0], ty - self.hw.pos[1]) < 0.35:
                if self.returning_vias:
                    self.returning_via = self.returning_vias.pop(0)
                    tx, ty = self.returning_via
                else:
                    self.returning_via = None
                    tx, ty = PREDOCK_X, PREDOCK_Y
        else:
            tx, ty = PREDOCK_X, PREDOCK_Y

        dx = tx - self.hw.pos[0]
        dy = ty - self.hw.pos[1]

        if self.returning_via is None:
            ret_elapsed = t - self.returning_start_time
            if ret_elapsed > 120.0:
                tol_x, tol_y = 0.15, 0.40
            elif ret_elapsed > 60.0:
                tol_x, tol_y = 0.09, 0.32
            else:
                tol_x, tol_y = 0.06, 0.25

            dx_abs = abs(PREDOCK_X - self.hw.pos[0])
            dy_abs = abs(PREDOCK_Y - self.hw.pos[1])
            if dx_abs < tol_x and dy_abs < tol_y:
                print(
                    f"[STATE CHANGE] RETURNING → ALIGNING "
                    f"(X={self.hw.pos[0]:.3f}, Y={self.hw.pos[1]:.3f}, "
                    f"ΔX={dx_abs:.3f}m, t_ret={ret_elapsed:.0f}s)"
                )
                self.returning_start_time = 0.0
                self.returning_via = None
                return True

        target_deg = math.degrees(math.atan2(dx, dy))
        if target_deg < 0.0:
            target_deg += 360.0
        err = angle_error(target_deg, self.hw.bearing)
        turn = clamp(0.045 * err, -0.40 * MAX_SPEED, 0.40 * MAX_SPEED)

        obs_front = self.hw.d_front < OBSTACLE_DIST
        obs_front_left = self.hw.d_front_left < OBSTACLE_DIST
        obs_front_right = self.hw.d_front_right < OBSTACLE_DIST
        obs_far_left = self.hw.d_far_left < OBSTACLE_DIST
        obs_far_right = self.hw.d_far_right < OBSTACLE_DIST
        base = 0.40 * MAX_SPEED * clamp(1.0 - abs(err) / 50.0, 0.0, 1.0)

        if obs_front or (obs_front_left and obs_front_right):
            if self.hw.d_far_left >= self.hw.d_far_right:
                self.hw.set_motor_speeds(-0.20 * MAX_SPEED, 0.42 * MAX_SPEED)
            else:
                self.hw.set_motor_speeds(0.42 * MAX_SPEED, -0.20 * MAX_SPEED)
        elif obs_front_left:
            self.hw.set_motor_speeds(0.45 * MAX_SPEED, 0.10 * MAX_SPEED)
        elif obs_front_right:
            self.hw.set_motor_speeds(0.10 * MAX_SPEED, 0.45 * MAX_SPEED)
        elif obs_far_left and not obs_far_right:
            self.hw.set_motor_speeds(
                base + turn + 0.08 * MAX_SPEED, base - turn - 0.08 * MAX_SPEED
            )
        elif obs_far_right and not obs_far_left:
            self.hw.set_motor_speeds(
                base + turn - 0.08 * MAX_SPEED, base - turn + 0.08 * MAX_SPEED
            )
        else:
            self.hw.set_motor_speeds(base + turn, base - turn)

        return False