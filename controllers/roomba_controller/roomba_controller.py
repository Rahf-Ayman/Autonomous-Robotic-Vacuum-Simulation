"""Autonomous vacuum controller for the Webots digital-twin simulation.

The controller coordinates coverage navigation, obstacle and cliff avoidance,
carpet-aware cleaning, battery management, and autonomous docking.
"""

import math
import random

try:
    from controller import Supervisor
except ImportError:
    from controller import Robot as Supervisor

from config import (
    ALIGNING_THRESHOLD,
    ALIGNING_TIMEOUT,
    BATTERY_LOW_PCT,
    BATTERY_MAX,
    BOUS_WP_RADIUS,
    CHARGER_X,
    CHARGER_Y,
    CHARGING_MAX_TIME,
    DOCKING_TIMEOUT,
    DOCK_ARM_Y,
    DOCK_CONFIRM_Y,
    DOCK_STALL_RADIUS_Y,
    MAX_SPEED,
    ROAMING_DURATION,
    STUCK_DIST,
    STUCK_TIMEOUT,
    angle_error,
    clamp,
)
from cleaning_manager import CleaningManager
from hardware import HardwareInterface
from navigation import NavigationPlanner
from pid import PIDController


class RoombaController:
    """Coordinate mission state machine, safety checks, and docking."""

    CLEANING_STATES = ("VACUUMING", "MOPPING")

    def __init__(self, robot, timestep):
        self.robot = robot
        self.timestep = timestep
        self.dt = timestep / 1000.0

        self.hw = HardwareInterface(robot, timestep)
        self.nav = NavigationPlanner(self.hw, self.dt)

        self.state = "UNDOCKING"
        self.prev_state = "VACUUMING"
        self.mission_phase = "VACUUMING"
        self.roaming_start_time = 0.0

        self.undock_phase = "REVERSE"
        self.undock_timer = 0.0

        # ESCAPE
        self.escape_reverse_end = 0.0
        self.escape_end_time = 0.0
        self.turn_direction = 1
        self.evasion_count = 0
        self.last_evasion_time = 0.0

        self.phases_completed = 0
        self.fresh_phase = True

        self.stuck_check_pos = None
        self.stuck_check_time = 0.0

        # PID for heading alignment during docking
        self.pid_bearing = PIDController(
            kp=0.045,
            ki=0.008,
            kd=0.003,
            setpoint=0.0,
            output_max=0.45 * MAX_SPEED,
            output_min=-0.45 * MAX_SPEED,
        )

        self.aligning_start_time = 0.0
        self.docking_start_time = 0.0
        self.dock_last_y = 0.0
        self.dock_stall_timer = 0.0

        self.charging_start_time = 0.0
        self.last_charge_time = None

        self.last_logged_pct = 110
        self.last_escape_log_t = -99.0
        self.escape_log_count = 0
        self.prev_cycle_time = 0.0

        self.hw.set_leds("UNDOCKING")
        self.cleaning_manager = CleaningManager(robot)
        self.cleaning_manager.init_environment()
        print(
            f"[SYSTEM] Controller v5.16 | {BATTERY_MAX:.0f} J battery | "
            f"return at {BATTERY_LOW_PCT * 100:.0f}% | 20 min per phase"
        )
        print(
            f"[MISSION] Starting phase 1: {self.mission_phase} | "
            f"{len(self.nav.bous_waypoints)} waypoints | "
            f"WP radius: {BOUS_WP_RADIUS}m"
        )

    def _watchdog(self, t):
        elapsed = t - self.prev_cycle_time
        if elapsed > 0.150 and self.prev_cycle_time > 0:
            print(f"[WATCHDOG] Cycle time exceeded: {elapsed:.3f}s")
        self.prev_cycle_time = t

    def _check_emergencies(self, t):
        if self.state not in (*self.CLEANING_STATES, "RETURNING"):
            return

        trigger = ""
        if self.hw.bumper_hit > 0.0:
            trigger = "BUMPER"
        elif self.hw.cliff_hit:
            trigger = "CLIFF"
        if not trigger:
            return

        battery_low = self.hw.battery < (BATTERY_LOW_PCT * BATTERY_MAX)
        time_up = (t - self.roaming_start_time) >= ROAMING_DURATION
        self.prev_state = "RETURNING" if (battery_low or time_up) else self.state

        self.nav.in_carpet_escape = False

        self.escape_log_count += 1
        if t - self.last_escape_log_t > 30.0:
            count_str = (
                f" ×{self.escape_log_count}" if self.escape_log_count > 1 else ""
            )
            print(f"[ESCAPE] {trigger}{count_str} in {self.state}")
            self.last_escape_log_t = t
            self.escape_log_count = 0
        self.state = "ESCAPE"
        self.hw.set_leds("ESCAPE")

        if t - self.last_evasion_time < 6.0:
            self.evasion_count += 1
        else:
            self.evasion_count = 1
        self.last_evasion_time = t

        if self.evasion_count >= 3:
            print("[WATCHDOG] Corner situation → 180° rotation.")
            self.escape_reverse_end = t + 1.0
            self.escape_end_time = self.escape_reverse_end + 2.2
            self.turn_direction = 1
            self.evasion_count = 0
        else:
            self.escape_reverse_end = t + 1.0
            self.escape_end_time = self.escape_reverse_end + random.uniform(0.8, 1.6)
            if (
                self.hw.wall_val > 200.0
                or self.hw.d_far_right < self.hw.d_far_left
            ):
                self.turn_direction = -1
            else:
                self.turn_direction = 1

    def _check_stuck(self, t):
        if self.state not in self.CLEANING_STATES:
            self.stuck_check_pos = None
            self.stuck_check_time = 0.0
            return

        if self.stuck_check_pos is None:
            self.stuck_check_pos = (self.hw.pos[0], self.hw.pos[1])
            self.stuck_check_time = t
            return

        moved = math.hypot(
            self.hw.pos[0] - self.stuck_check_pos[0],
            self.hw.pos[1] - self.stuck_check_pos[1],
        )
        if moved > STUCK_DIST:
            self.stuck_check_pos = (self.hw.pos[0], self.hw.pos[1])
            self.stuck_check_time = t
        elif t - self.stuck_check_time > STUCK_TIMEOUT:
            print(
                f"[STUCK] No {STUCK_DIST}m movement in {STUCK_TIMEOUT:.0f}s "
                f"(pos={self.hw.pos[0]:.2f},{self.hw.pos[1]:.2f}) → ESCAPE spin"
            )
            self.prev_state = self.state
            self.state = "ESCAPE"
            self.escape_reverse_end = t + 0.3
            self.escape_end_time = t + 0.3 + 1.5
            self.turn_direction = (
                1 if self.hw.d_far_left >= self.hw.d_far_right else -1
            )
            self.nav.cleaning_speed = 0.0
            self.nav.wp_progress_time = 0.0
            self.nav.wp_prev_dist = float("inf")
            self.stuck_check_pos = None
            self.stuck_check_time = 0.0
            self.hw.set_leds("ESCAPE")

    def _transition_to_returning(self):
        self.pid_bearing.reset()
        self.nav.in_carpet_escape = False
        self.nav.returning_start_time = 0.0
        self.nav.cleaning_speed = 0.0
        self.state = "RETURNING"
        self.hw.set_leds("RETURNING")

    def _enter_charging(self):
        """Stop at the dock and initialize the charging state."""
        docked_by_bumper = (
            self.hw.bumper_hit > 0 and self.hw.pos[1] > DOCK_ARM_Y
        )
        trigger = (
            "bumper"
            if docked_by_bumper
            else f"GPS/stall Y={self.hw.pos[1]:.2f}"
        )
        distance = abs(CHARGER_Y - self.hw.pos[1])
        print(
            f"[STATE CHANGE] DOCKING → CHARGING ({trigger}, distance={distance:.2f}m)"
        )
        self.last_charge_time = self.robot.getTime()
        self.charging_start_time = 0.0
        self.docking_start_time = 0.0
        self.dock_last_y = 0.0
        self.state = "CHARGING"
        self.hw.set_leds("CHARGING")
        self.hw.set_motor_speeds(0.0, 0.0)

    def _complete_charging(self, battery_percentage):
        """Resume the current phase or transition to the next mission phase."""
        waypoint_count = len(self.nav.bous_waypoints)
        completed_count = len(self.nav.wp_done[self.mission_phase])
        current_time = self.robot.getTime()
        print(f"[BATTERY] Charged: {battery_percentage}%")
        self.last_logged_pct = battery_percentage
        self.nav.cleaning_via = None
        self.nav.cleaning_vias = []

        if completed_count >= waypoint_count:
            self.phases_completed += 1
            previous_phase = self.mission_phase

            if self.phases_completed >= len(self.CLEANING_STATES):
                print("[STATE CHANGE] CHARGING → FINISHED")
                print(
                    "[MISSION] Complete! VACUUMING + MOPPING done "
                    f"| t={current_time:.0f}s | battery: {battery_percentage}%"
                )
                self.cleaning_manager.print_summary()
                self.last_charge_time = current_time
                self.charging_start_time = 0.0
                self.state = "FINISHED"
                self.hw.set_leds("FINISHED")
                self.hw.set_motor_speeds(0.0, 0.0)
                return

            self.mission_phase = (
                "MOPPING" if previous_phase == "VACUUMING" else "VACUUMING"
            )
            self.nav.wp_done[self.mission_phase].clear()
            self.fresh_phase = True
            self.nav.bous_wp_idx = 0
            self.nav.bous_waypoints = (
                self.nav.generate_mopping_waypoints()
                if self.mission_phase == "MOPPING"
                else self.nav.generate_waypoints()
            )
            print(
                "[STATE CHANGE] CHARGING → UNDOCKING "
                f"| {previous_phase} done → next: {self.mission_phase} "
                f"| battery: {battery_percentage}% "
                f"| [{self.phases_completed}/{len(self.CLEANING_STATES)} phases]"
            )
        else:
            remaining = waypoint_count - completed_count
            if self.mission_phase == "MOPPING" and completed_count >= 4:
                remaining_indices = [
                    i
                    for i in range(waypoint_count)
                    if i not in self.nav.wp_done[self.mission_phase]
                ]
                self.nav.bous_wp_idx = remaining_indices[0]
                target_x, target_y = self.nav.bous_waypoints[self.nav.bous_wp_idx]
                vias = [(-2.65, 1.80)]
                if target_y > 0.45:
                    vias.append((target_x, target_y))
                elif target_y >= -0.75:
                    vias.extend([(-1.10, 0.75), (-1.10, target_y)])
                else:
                    vias.extend([(-1.10, 0.75), (-1.10, -1.00)])
                    if target_x < -1.10:
                        vias.append((target_x, target_y))
                    elif target_y < -1.00:
                        vias.append((-1.10, target_y))
                self.nav.cleaning_via = vias[0]
                self.nav.cleaning_vias = vias[1:]
            self.fresh_phase = False
            print(
                "[STATE CHANGE] CHARGING → UNDOCKING "
                f"| {self.mission_phase}: {completed_count}/{waypoint_count} WP done, "
                f"{remaining} remaining | battery: {battery_percentage}%"
            )

        self.last_charge_time = current_time
        self.charging_start_time = 0.0
        self.state = "UNDOCKING"
        self.undock_phase = "REVERSE"
        self.undock_timer = 0.0
        self.nav.in_carpet_escape = False
        self.hw.set_leds("UNDOCKING")

    def _execute_cleaning_phase(self, t, allow_carpet):
        battery_low = self.hw.battery < (BATTERY_LOW_PCT * BATTERY_MAX)
        time_up = (t - self.roaming_start_time) >= ROAMING_DURATION

        if battery_low or time_up:
            reason = (
                "time limit 20 min"
                if time_up
                else f"low battery ({BATTERY_LOW_PCT * 100:.0f}%)"
            )
            print(f"[STATE CHANGE] {self.state} → RETURNING ({reason})")
            self.cleaning_manager.print_summary()
            self._transition_to_returning()
            return

        if self.nav.step_cleaning(t, allow_carpet):
            self.cleaning_manager.print_summary()
            self._transition_to_returning()

    def _execute_state_machine(self, t):
        # ── UNDOCKING ─────────────────────────────────────────────────────────
        if self.state == "UNDOCKING":
            if self.undock_timer == 0.0:
                self.undock_timer = t

            if self.undock_phase == "REVERSE":
                self.hw.set_motor_speeds(-0.45 * MAX_SPEED, -0.45 * MAX_SPEED)
                if self.hw.pos[1] < 1.80 or (t - self.undock_timer > 4.0):
                    self.undock_phase = "TURN"
                    self.undock_timer = t

            elif self.undock_phase == "TURN":
                err = angle_error(180.0, self.hw.bearing)
                if abs(err) < 2.5 or (t - self.undock_timer > 5.0):
                    self.roaming_start_time = t
                    self.nav.returning_start_time = 0.0
                    self.nav.returning_via = None
                    self.nav.in_carpet_escape = False
                    self.nav.cleaning_speed = 0.0
                    self.nav.wp_progress_time = 0.0
                    self.nav.wp_prev_dist = float("inf")

                    keep_mopping_via = (
                        self.mission_phase == "MOPPING" and not self.fresh_phase
                    )
                    if not keep_mopping_via:
                        phase_done = self.nav.wp_done[self.mission_phase]
                        n_wps = len(self.nav.bous_waypoints)
                        found_next = False
                        for i in range(n_wps):
                            idx = (self.nav.bous_wp_idx + i) % n_wps
                            if idx not in phase_done:
                                self.nav.bous_wp_idx = idx
                                found_next = True
                                break
                        if not found_next:
                            self.nav.bous_wp_idx = 0

                    self.nav.exploration_factor = 0.50 if self.fresh_phase else 0.85
                    self.fresh_phase = False

                    self.nav.wp_skipped_count = 0
                    self.state = self.mission_phase
                    self.hw.set_leds(self.mission_phase)
                    print(
                        f"[STATE CHANGE] UNDOCKING → {self.mission_phase} "
                        f"(t={t:.0f}s | WP#{self.nav.bous_wp_idx}/{len(self.nav.bous_waypoints)})"
                    )
                else:
                    turn_sp = clamp(
                        0.04 * abs(err), 0.08 * MAX_SPEED, 0.40 * MAX_SPEED
                    )
                    if err > 0:
                        self.hw.set_motor_speeds(turn_sp, -turn_sp)
                    else:
                        self.hw.set_motor_speeds(-turn_sp, turn_sp)

        # ── VACUUMING ─────────────────────────────────────────────────────────
        elif self.state == "VACUUMING":
            self._execute_cleaning_phase(t, allow_carpet=True)

        # ── MOPPING ───────────────────────────────────────────────────────────
        elif self.state == "MOPPING":
            self._execute_cleaning_phase(t, allow_carpet=False)

        # ── ESCAPE ────────────────────────────────────────────────────────────
        elif self.state == "ESCAPE":
            if t < self.escape_reverse_end:
                self.hw.set_motor_speeds(-0.50 * MAX_SPEED, -0.50 * MAX_SPEED)
            elif t < self.escape_end_time:
                td = self.turn_direction
                self.hw.set_motor_speeds(
                    td * 0.45 * MAX_SPEED, -td * 0.45 * MAX_SPEED
                )
            else:
                self.nav.cleaning_speed = 0.0
                self.nav.wp_progress_time = 0.0
                self.nav.wp_prev_dist = float("inf")
                self.state = self.prev_state
                self.hw.set_leds(self.prev_state)

        # ── RETURNING ─────────────────────────────────────────────────────────
        elif self.state == "RETURNING":
            if self.nav.step_returning(t):
                self.aligning_start_time = t
                self.pid_bearing.reset(setpoint=0.0)
                self.state = "ALIGNING"
                self.hw.set_leds("ALIGNING")

        # ── ALIGNING ──────────────────────────────────────────────────────────
        elif self.state == "ALIGNING":
            err = angle_error(0.0, self.hw.bearing)
            aligned = abs(err) < ALIGNING_THRESHOLD
            timed_out = (t - self.aligning_start_time) > ALIGNING_TIMEOUT

            if aligned or timed_out:
                if timed_out and not aligned:
                    print(f"[ALIGNING] Timeout → DOCKING (err={err:.1f}°)")
                else:
                    print(f"[STATE CHANGE] ALIGNING → DOCKING (err={err:.1f}°)")
                self.aligning_start_time = 0.0
                self.docking_start_time = t
                self.dock_last_y = 0.0
                self.dock_stall_timer = t
                self.pid_bearing.reset()
                self.state = "DOCKING"
                self.hw.set_leds("DOCKING")
            else:
                turn = self.pid_bearing.compute_error(err, self.dt)
                if abs(turn) < 0.02 * MAX_SPEED:
                    turn = math.copysign(0.02 * MAX_SPEED, err)
                self.hw.set_motor_speeds(turn, -turn)

        # ── DOCKING ───────────────────────────────────────────────────────────
        elif self.state == "DOCKING":
            if self.dock_last_y == 0.0:
                self.dock_last_y = self.hw.pos[1]
                self.dock_stall_timer = t
            if self.hw.pos[1] > self.dock_last_y + 0.02:
                self.dock_last_y = self.hw.pos[1]
                self.dock_stall_timer = t

            bumper_in_dock = (
                self.hw.bumper_hit > 0.0 and self.hw.pos[1] > DOCK_ARM_Y
            )
            gps_in_dock = self.hw.pos[1] > DOCK_CONFIRM_Y
            stalled = (t - self.dock_stall_timer) > 8.0

            if bumper_in_dock or gps_in_dock:
                self._enter_charging()
            elif stalled:
                if self.hw.pos[1] > DOCK_STALL_RADIUS_Y:
                    self._enter_charging()
                else:
                    print(
                        f"[DOCKING] Stall outside radius (Y={self.hw.pos[1]:.2f}) → ALIGNING"
                    )
                    self.docking_start_time = 0.0
                    self.dock_last_y = 0.0
                    self.aligning_start_time = t
                    self.state = "ALIGNING"
                    self.hw.set_leds("ALIGNING")
                    self.hw.set_motor_speeds(0.0, 0.0)
            elif (t - self.docking_start_time) > DOCKING_TIMEOUT:
                print(f"[DOCKING] Timeout → ALIGNING (Y={self.hw.pos[1]:.2f})")
                self.docking_start_time = 0.0
                self.dock_last_y = 0.0
                self.aligning_start_time = t
                self.state = "ALIGNING"
                self.hw.set_leds("ALIGNING")
                self.hw.set_motor_speeds(0.0, 0.0)
            else:
                x_err = CHARGER_X - self.hw.pos[0]
                bearing_err = angle_error(0.0, self.hw.bearing)
                x_turn = clamp(4.0 * x_err, -0.20 * MAX_SPEED, 0.20 * MAX_SPEED)
                b_turn = clamp(0.02 * bearing_err, -0.08 * MAX_SPEED, 0.08 * MAX_SPEED)
                turn = clamp(x_turn + b_turn, -0.20 * MAX_SPEED, 0.20 * MAX_SPEED)
                self.hw.set_motor_speeds(
                    0.12 * MAX_SPEED + turn, 0.12 * MAX_SPEED - turn
                )

        # ── CHARGING ──────────────────────────────────────────────────────────
        elif self.state == "CHARGING":
            if self.charging_start_time == 0.0:
                self.charging_start_time = t

            if self.hw.pos[1] < DOCK_STALL_RADIUS_Y:
                print(
                    f"[CHARGING] Outside charging zone (Y={self.hw.pos[1]:.2f}) → DOCKING"
                )
                self.docking_start_time = t
                self.dock_last_y = 0.0
                self.dock_stall_timer = t
                self.charging_start_time = 0.0
                self.state = "DOCKING"
                self.hw.set_leds("DOCKING")
                return

            self.hw.set_motor_speeds(0.0, 0.0)
            pct = (
                int((self.hw.battery / BATTERY_MAX) * 100)
                if self.hw.battery >= 0
                else 0
            )

            if pct >= 95:
                self._complete_charging(pct)
            elif (t - self.charging_start_time) > CHARGING_MAX_TIME:
                print(f"[CHARGING] Timeout {CHARGING_MAX_TIME:.0f}s (battery: {pct}%)")
                self._complete_charging(pct)

        # ── FINISHED ──────────────────────────────────────────────────────────
        elif self.state == "FINISHED":
            self.hw.set_motor_speeds(0.0, 0.0)

    def run_step(self):
        t = self.robot.getTime()
        self._watchdog(t)
        pct = self.hw.read_sensors(self.state, self.nav.carpet_map)

        active = self.state in (
            "VACUUMING",
            "MOPPING",
            "RETURNING",
            "ESCAPE",
            "UNDOCKING",
        )
        if (
            active
            and pct != self.last_logged_pct
            and pct in (50, 20)
            and 0 <= pct <= 100
        ):
            print(f"[BATTERY] {pct}%")
            self.last_logged_pct = pct

        self._check_emergencies(t)
        self._check_stuck(t)
        self.cleaning_manager.step(self.hw.pos[0], self.hw.pos[1], self.state)
        self._execute_state_machine(t)


def main():
    """Run the controller in the active Webots simulation."""
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    controller = RoombaController(robot, timestep)

    while robot.step(timestep) != -1:
        controller.run_step()


if __name__ == "__main__":
    main()