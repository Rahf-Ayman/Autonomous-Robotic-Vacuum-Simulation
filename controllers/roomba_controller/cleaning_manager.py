"""Environmental cleaning manager for dry dust (vacuuming) and liquid juice (mopping)."""

import math
import random

# Walkable bounds and keep-out parameters (m)
CLEANING_X_MIN = -2.50
CLEANING_X_MAX = 2.50
CLEANING_Y_MIN = -1.90
CLEANING_Y_MAX = 2.05
DUST_STEP = 0.36
DUST_JITTER = 0.06

# Suction intake & mopping pad parameters (m)
SUCTION_RADIUS = 0.17  # Matches the Roomba chassis radius
MOP_CONTACT_RADIUS = 0.21  # Effective coverage radius for mopping pad

# Carpet boundary estimates for height adjustment
CARPET_X_MIN = 0.14
CARPET_X_MAX = 1.74
CARPET_Y_MIN = -0.99
CARPET_Y_MAX = 1.41

# Number of random juice puddles for mopping
NUM_JUICE_SPILLS = 6


class CleaningManager:
    """Manage environmental cleaning entities: dry dust specks and liquid juice puddles."""

    def __init__(self, supervisor):
        self.supervisor = supervisor
        self.enabled = False

        # ── Dry Dust Particles (Vacuuming) ────────────────────────────────────
        self.dust_particles = {}  # id -> {"x", "y", "z", "node", "on_carpet"}
        self.total_dust = 0
        self.dust_cleaned = 0
        self.floor_dust_cleaned = 0
        self.carpet_dust_cleaned = 0
        self.last_reported_dust_pct = 0

        # ── Liquid Juice Spills (Mopping) ─────────────────────────────────────
        self.juice_spills = {}  # id -> {"x", "y", "radius", "node"}
        self.total_juice = 0
        self.juice_cleaned = 0

    def is_walkable(self, x, y):
        """Check if (x, y) is inside the walkable apartment area avoiding furniture."""
        if not (CLEANING_X_MIN <= x <= CLEANING_X_MAX and CLEANING_Y_MIN <= y <= CLEANING_Y_MAX):
            return False

        # Desk and office chair (west wall area)
        if -0.80 <= y <= 0.50 and x < -0.95:
            return False

        # Bed and bedside approaches (east nook)
        if -1.05 <= y <= 1.45 and x > 1.65:
            return False

        # Charger docking sanctuary
        if math.hypot(x - 2.00, y - 2.85) < 0.75:
            return False

        # Sofa boundary (south)
        if y < -2.10:
            return False

        # Kitchen worktop & sink boundary (north)
        if y > 2.15:
            return False

        return True

    def is_on_carpet(self, x, y, buffer=0.0):
        """Check whether coordinate (x, y) is over or within buffer of the carpet/rug."""
        return (
            (CARPET_X_MIN - buffer) <= x <= (CARPET_X_MAX + buffer)
            and (CARPET_Y_MIN - buffer) <= y <= (CARPET_Y_MAX + buffer)
        )

    def init_environment(self):
        """Initialize both dust specks (for vacuuming) and juice spills (for mopping)."""
        self.init_dust_field()
        self.init_juice_field()

    def init_dust_field(self):
        """Procedurally spawn dust specks across the apartment floor and carpet."""
        try:
            dust_field_node = self.supervisor.getFromDef("DUST_FIELD")
            if not dust_field_node:
                print("[CLEANING] Warning: DUST_FIELD group not found in world. Dust disabled.")
                return
            children_field = dust_field_node.getField("children")
            if not children_field:
                print("[CLEANING] Warning: Could not access children field of DUST_FIELD.")
                return
        except Exception as e:
            print(f"[CLEANING] Supervisor API unavailable ({e}). Running without visual dust.")
            return

        while children_field.getCount() > 0:
            children_field.removeMF(0)

        self.dust_particles.clear()
        self.total_dust = 0
        self.dust_cleaned = 0
        self.floor_dust_cleaned = 0
        self.carpet_dust_cleaned = 0
        self.last_reported_dust_pct = 0

        random.seed(42)

        y = CLEANING_Y_MAX
        idx = 0
        n_floor = 0
        n_carpet = 0

        while y >= CLEANING_Y_MIN - 0.01:
            x = CLEANING_X_MIN
            while x <= CLEANING_X_MAX + 0.01:
                jx = x + random.uniform(-DUST_JITTER, DUST_JITTER)
                jy = y + random.uniform(-DUST_JITTER, DUST_JITTER)

                if self.is_walkable(jx, jy):
                    on_carpet = self.is_on_carpet(jx, jy)
                    jz = 0.008 if on_carpet else 0.002

                    def_name = f"DUST_{idx}"
                    node_str = (
                        f"DEF {def_name} Solid {{\n"
                        f"  translation {jx:.3f} {jy:.3f} {jz:.3f}\n"
                        f"  children [\n"
                        f"    Shape {{\n"
                        f"      appearance PBRAppearance {{\n"
                        f"        baseColor 0.28 0.24 0.20\n"
                        f"        roughness 0.95\n"
                        f"        metalness 0.0\n"
                        f"      }}\n"
                        f"      geometry Cylinder {{\n"
                        f"        height 0.002\n"
                        f"        radius 0.024\n"
                        f"      }}\n"
                        f"    }}\n"
                        f"  ]\n"
                        f"}}"
                    )

                    children_field.importMFNodeFromString(-1, node_str)
                    node = self.supervisor.getFromDef(def_name)

                    if node:
                        self.dust_particles[idx] = {
                            "x": jx,
                            "y": jy,
                            "z": jz,
                            "node": node,
                            "on_carpet": on_carpet,
                        }
                        if on_carpet:
                            n_carpet += 1
                        else:
                            n_floor += 1
                        idx += 1

                x += DUST_STEP
            y -= DUST_STEP

        self.total_dust = len(self.dust_particles)
        self.enabled = self.total_dust > 0

        print(
            f"[CLEANING] Procedurally spawned {self.total_dust} dust atoms "
            f"({n_floor} on hard floor, {n_carpet} on carpet | suction radius={SUCTION_RADIUS}m)"
        )

    def init_juice_field(self):
        """Procedurally spawn glossy orange juice puddles exclusively on the hard floor."""
        try:
            juice_field_node = self.supervisor.getFromDef("JUICE_FIELD")
            if not juice_field_node:
                juice_field_node = self.supervisor.getFromDef("DUST_FIELD")
            if not juice_field_node:
                return
            children_field = juice_field_node.getField("children")
            if not children_field:
                return
        except Exception:
            return

        # If JUICE_FIELD was found as a dedicated group, clear its children
        if juice_field_node.getDef() == "JUICE_FIELD":
            while children_field.getCount() > 0:
                children_field.removeMF(0)

        self.juice_spills.clear()
        self.total_juice = 0
        self.juice_cleaned = 0

        # Candidate zones for juice spills on the hard floor
        random.seed(101)

        candidate_regions = [
            (-1.80, 1.60),   # Northwest near kitchen
            (-0.80, 1.85),   # North kitchen corridor
            (-1.60, 0.50),   # West central floor
            (-1.20, -0.20),  # Central western corridor
            (-1.70, -1.20),  # Southwest near desk/sofa
            (-0.40, -1.50),  # South floor in front of sofa
            (0.00, 1.80),    # North transition strip
        ]

        puddle_idx = 0
        for (cx, cy) in candidate_regions:
            if puddle_idx >= NUM_JUICE_SPILLS:
                break

            jx = cx + random.uniform(-0.15, 0.15)
            jy = cy + random.uniform(-0.15, 0.15)

            if self.is_walkable(jx, jy) and not self.is_on_carpet(jx, jy, buffer=0.30):
                radius = random.uniform(0.065, 0.090)
                def_name = f"JUICE_{puddle_idx}"

                node_str = (
                    f"DEF {def_name} Solid {{\n"
                    f"  translation {jx:.3f} {jy:.3f} 0.0025\n"
                    f"  children [\n"
                    f"    Shape {{\n"
                    f"      appearance PBRAppearance {{\n"
                    f"        baseColor 0.94 0.45 0.06\n"
                    f"        roughness 0.15\n"
                    f"        metalness 0.05\n"
                    f"        transparency 0.10\n"
                    f"      }}\n"
                    f"      geometry Cylinder {{\n"
                    f"        height 0.002\n"
                    f"        radius {radius:.3f}\n"
                    f"      }}\n"
                    f"    }}\n"
                    f"  ]\n"
                    f"}}"
                )

                children_field.importMFNodeFromString(-1, node_str)
                node = self.supervisor.getFromDef(def_name)

                if node:
                    self.juice_spills[puddle_idx] = {
                        "x": jx,
                        "y": jy,
                        "radius": radius,
                        "node": node,
                    }
                    puddle_idx += 1

        self.total_juice = len(self.juice_spills)
        print(
            f"[CLEANING] Spawned {self.total_juice} liquid juice spills on hard floor "
            f"(awaiting mopping phase | glossy orange puddles Ø~15cm)"
        )

    def step(self, robot_x, robot_y, state):
        """Collect dust and mop juice puddles based on active cleaning state."""
        # ── 1. Dry Dust Suction (Active during VACUUMING, MOPPING, or ESCAPE) ──
        if self.enabled and self.dust_particles and state in ("VACUUMING", "MOPPING", "ESCAPE"):
            cleaned_now = []
            r2 = SUCTION_RADIUS * SUCTION_RADIUS

            for p_id, p in self.dust_particles.items():
                if abs(p["x"] - robot_x) > SUCTION_RADIUS or abs(p["y"] - robot_y) > SUCTION_RADIUS:
                    continue

                dx = p["x"] - robot_x
                dy = p["y"] - robot_y
                if (dx * dx + dy * dy) <= r2:
                    cleaned_now.append(p_id)

            for p_id in cleaned_now:
                p = self.dust_particles.pop(p_id)
                try:
                    p["node"].remove()
                except Exception:
                    pass

                self.dust_cleaned += 1
                if p["on_carpet"]:
                    self.carpet_dust_cleaned += 1
                else:
                    self.floor_dust_cleaned += 1

            if self.total_dust > 0:
                pct = int((self.dust_cleaned / self.total_dust) * 100)
                if pct >= self.last_reported_dust_pct + 10:
                    self.last_reported_dust_pct = (pct // 10) * 10
                    print(
                        f"[DUST] Progress: {self.dust_cleaned}/{self.total_dust} "
                        f"({pct}%) collected | Floor: {self.floor_dust_cleaned}, Carpet: {self.carpet_dust_cleaned}"
                    )

        # ── 2. Wet Juice Mopping (Active specifically during MOPPING) ──────────
        if self.juice_spills and state in ("MOPPING", "ESCAPE"):
            mopped_now = []
            for j_id, j in self.juice_spills.items():
                reach = MOP_CONTACT_RADIUS + j["radius"] * 0.5
                if abs(j["x"] - robot_x) > reach or abs(j["y"] - robot_y) > reach:
                    continue

                dx = j["x"] - robot_x
                dy = j["y"] - robot_y
                if (dx * dx + dy * dy) <= (reach * reach):
                    mopped_now.append(j_id)

            for j_id in mopped_now:
                j = self.juice_spills.pop(j_id)
                try:
                    j["node"].remove()
                except Exception:
                    pass

                self.juice_cleaned += 1
                print(
                    f"[MOPPING] Wiped juice puddle #{j_id + 1} at ({j['x']:.2f}, {j['y']:.2f}) "
                    f"| Liquid spills cleaned: {self.juice_cleaned}/{self.total_juice}"
                )

    def print_summary(self):
        """Print detailed summary of cleaning performance."""
        if self.total_dust > 0:
            pct = (self.dust_cleaned / self.total_dust) * 100
            print(
                f"[DUST REPORT] Cleaned: {self.dust_cleaned}/{self.total_dust} ({pct:.1f}%) "
                f"[Hard Floor: {self.floor_dust_cleaned}, Carpet: {self.carpet_dust_cleaned}, "
                f"Remaining: {len(self.dust_particles)}]"
            )
        if self.total_juice > 0:
            jpct = (self.juice_cleaned / self.total_juice) * 100
            print(
                f"[JUICE REPORT] Mopped: {self.juice_cleaned}/{self.total_juice} ({jpct:.1f}%) "
                f"liquid spills [Remaining: {len(self.juice_spills)}]"
            )
