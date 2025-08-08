# Find your printers max speed before losing steps
#
# Copyright (C) 2024 Anonoei <dev@anonoei.com>
# This file contains extensive modifications to the acceleration testing methodology
# based on a new testing philosophy for improved accuracy, safety, and reliability.
#
# This file may be distributed under the terms of the MIT license.

import os
from time import perf_counter
import datetime as dt
import math

from .funcs import calculate_graph, calculate_accel_focused_dist
from .move import Move, MoveX, MoveY, MoveZ, MoveDiagX, MoveDiagY
from .wrappers import ResultsWrapper, AttemptWrapper

class AutoSpeed:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode_move = self.printer.load_object(config, 'gcode_move')

        self.printer_kinematics = self.config.getsection("printer").get("kinematics")
        self.isolate_xy = self.printer_kinematics == 'cartesian' or self.printer_kinematics == 'corexz'

        self.valid_axes = ["x", "y", "diag_x", "diag_y", "z"]
        self.axes = self._parse_axis(config.get('axis', 'x, y' if self.isolate_xy else 'diag_x, diag_y'))

        self.default_axes = ''
        for axis in self.axes:
            self.default_axes += f"{axis},"
        self.default_axes = self.default_axes[:-1]

        self.margin = config.getfloat('margin', default=20.0, above=0.0)
        self.settling_home = config.getboolean('settling_home', default=True)
        self.max_missed_original = config.getfloat('max_missed', default=1.0)
        self.endstop_samples = config.getint('endstop_samples', default=3, minval=2)

        self.accel_min = config.getfloat('accel_min', default=1000.0, above=1.0)
        self.accel_max = config.getfloat('accel_max', default=100000.0, above=self.accel_min)
        self.accel_accu = config.getfloat('accel_accu', default=0.05, above=0.0, below=1.0)
        self.scv = config.getfloat('scv', default=5, above=1.0, below=50)

        self.veloc_min = config.getfloat('velocity_min', default=50.0, above=1.0)
        self.veloc_max = config.getfloat('velocity_max', default=5000.0, above=self.veloc_min)
        self.veloc_accu = config.getfloat('velocity_accu', default=0.05, above=0.0, below=1.0)

        self.derate = config.getfloat('derate', default=0.8, above=0.0, below=1.0)
        
        self.accel_test_velocity = config.getfloat('accel_test_velocity', 200.0, above=1.0)
        self.samples_per_test_type = config.getint('samples_per_test_type', 3, minval=1)
        self.final_validation_iterations = config.getint('final_validation_iterations', 5, minval=1)
        self.validation_pattern_size = config.getfloat('validation_pattern_size', None, above=0.0)

        self.MAX_MISSED_THRESHOLD = 3.0
        self.MIN_SHORT_MOVE_DISTANCE = 5.0

        results_default = os.path.expanduser('~')
        for path in (os.path.dirname(self.printer.start_args['log_file']), os.path.expanduser('~/printer_data/config')):
            if os.path.exists(path):
                results_default = path
        self.results_dir = os.path.expanduser(config.get('results_dir', default=results_default))

        self.toolhead = None
        self.default_velocity = None
        self.default_accel = None
        self.default_scv = None
        self.default_accel_control_val = None
        self.use_cruise_ratio = False
        
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.printer.register_event_handler("homing:home_rails_end", self.handle_home_rails_end)

        self.gcode.register_command('AUTO_SPEED', self.cmd_AUTO_SPEED, desc=self.cmd_AUTO_SPEED_help)
        self.gcode.register_command('AUTO_SPEED_VELOCITY', self.cmd_AUTO_SPEED_VELOCITY, desc=self.cmd_AUTO_SPEED_VELOCITY_help)
        self.gcode.register_command('AUTO_SPEED_ACCEL', self.cmd_AUTO_SPEED_ACCEL, desc=self.cmd_AUTO_SPEED_ACCEL_help)
        self.gcode.register_command('AUTO_SPEED_VALIDATE', self.cmd_AUTO_SPEED_VALIDATE, desc=self.cmd_AUTO_SPEED_VALIDATE_help)
        self.gcode.register_command('AUTO_SPEED_GRAPH', self.cmd_AUTO_SPEED_GRAPH, desc=self.cmd_AUTO_SPEED_GRAPH_help)
        self.gcode.register_command('X_ENDSTOP_ACCURACY', self.cmd_X_ENDSTOP_ACCURACY, desc=self.cmd_AUTO_SPEED_GRAPH_help)
        self.gcode.register_command('Y_ENDSTOP_ACCURACY', self.cmd_Y_ENDSTOP_ACCURACY, desc=self.cmd_AUTO_SPEED_GRAPH_help)
        self.gcode.register_command('Z_ENDSTOP_ACCURACY', self.cmd_Z_ENDSTOP_ACCURACY, desc=self.cmd_AUTO_SPEED_GRAPH_help)

        self.level = None
        self.steppers = {}
        self.axis_limits = {}

    cmd_AUTO_SPEED_help = ("Performs a full, multi-stage characterization of the printer's motion system.")
    def cmd_AUTO_SPEED(self, gcmd):
        if not len(self.steppers.keys()) == 3: raise gcmd.error(f"Printer must be homed first!")
        
        axes = self._parse_axis(gcmd.get("AXIS", self.default_axes))
        accel_min = gcmd.get_float('ACCEL_MIN', self.accel_min, above=1.0)
        accel_max = gcmd.get_float('ACCEL_MAX', self.accel_max, above=accel_min)
        veloc_max = gcmd.get_float('VELOCITY_MAX', self.veloc_max, above=self.veloc_min)
        derate = gcmd.get_float('DERATE', self.derate, above=0.0, below=1.0)
        samples = gcmd.get_int('SAMPLES', self.samples_per_test_type, minval=1)
        validation_iters = gcmd.get_int('VALIDATION_ITERATIONS', self.final_validation_iterations, minval=1)

        self._prepare(gcmd)
        move_z = gcmd.get_int('Z', None)
        if move_z is not None: self._move([None, None, move_z], self.default_velocity)

        self.gcode.respond_info("\n" + "="*40)
        self.gcode.respond_info(f"STAGE 1: Finding baseline max acceleration on '{axes[0]}' at {self.accel_test_velocity}mm/s.")
        self.gcode.respond_info("="*40 + "\n")
        
        aw_accel = AttemptWrapper()
        aw_accel.type, aw_accel.accuracy, aw_accel.max_missed = "accel", self.accel_accu, self.MAX_MISSED_THRESHOLD
        aw_accel.margin, aw_accel.min, aw_accel.max, aw_accel.scv = self.margin, accel_min, accel_max, self.scv
        self.init_axis(aw_accel, axes[0])
        
        max_accel = self.accel_binary_search(aw_accel, samples)
        rec_accel = max_accel * derate
        self.gcode.respond_info(f"\nResult of Stage 1: Max Accel = {max_accel:.0f}, Recommended Safe Accel = {rec_accel:.0f}")

        self.gcode.respond_info("\n" + "="*40)
        self.gcode.respond_info(f"STAGE 2: Finding max velocity on '{axes[0]}' using safe accel ({rec_accel:.0f}mm/s^2).")
        self.gcode.respond_info("="*40 + "\n")

        aw_velo = AttemptWrapper()
        aw_velo.type, aw_velo.accuracy, aw_velo.max_missed = "velocity", self.veloc_accu, self.max_missed_original
        aw_velo.margin, aw_velo.min, aw_velo.max, aw_velo.scv = self.margin, self.veloc_min, veloc_max, self.scv
        aw_velo.accel = rec_accel
        self.init_axis(aw_velo, axes[0])
        max_velo = self.binary_search(aw_velo)
        rec_velo = max_velo * derate
        self.gcode.respond_info(f"\nResult of Stage 2: Max Velo = {max_velo:.0f}, Recommended Safe Velo = {rec_velo:.0f}")

        self.gcode.respond_info("\n" + "="*40)
        self.gcode.respond_info("STAGE 3: Characterizing high-speed acceleration performance.")
        self.gcode.respond_info("="*40 + "\n")
        gcmd.get_command_parameters()['AXIS'] = axes[0]
        self.cmd_AUTO_SPEED_GRAPH(gcmd)

        self.gcode.respond_info("\n" + "="*40)
        self.gcode.respond_info(f"STAGE 4: Performing final 'Chaos Star' validation of recommended accel ({rec_accel:.0f}mm/s^2).")
        self.gcode.respond_info("="*40 + "\n")
        self.run_safe_validation(rec_accel, validation_iters)

        self.gcode.respond_info("\n" + "="*40)
        self.gcode.respond_info("AUTO_SPEED Routine Complete.")
        self.gcode.respond_info(f"Final Recommended Settings: Accel={rec_accel:.0f}, Velocity={rec_velo:.0f}")
        self.gcode.respond_info("="*40)

    cmd_AUTO_SPEED_ACCEL_help = ("(Advanced) Finds max acceleration using the robust multi-test methodology.")
    def cmd_AUTO_SPEED_ACCEL(self, gcmd):
        if not len(self.steppers.keys()) == 3: raise gcmd.error(f"Printer must be homed first!")
        axes = self._parse_axis(gcmd.get("AXIS", self.default_axes))
        margin = gcmd.get_float("MARGIN", self.margin, above=0.0)
        derate = gcmd.get_float('DERATE', self.derate, above=0.0, below=1.0)
        accel_min = gcmd.get_float('ACCEL_MIN', self.accel_min, above=1.0)
        accel_max = gcmd.get_float('ACCEL_MAX', self.accel_max, above=accel_min)
        accel_accu = gcmd.get_float('ACCEL_ACCU', self.accel_accu, above=0.0, below=1.0)
        scv = gcmd.get_float('SCV', self.scv, above=1.0)
        samples = gcmd.get_int('SAMPLES', self.samples_per_test_type, minval=1)
        validation_iters = gcmd.get_int('VALIDATION_ITERATIONS', self.final_validation_iterations, minval=1)

        self.gcode.respond_info(f"Starting robust acceleration search on axes: {axes}")
        self.gcode.respond_info(f"Using robust failure threshold: max_missed = {self.MAX_MISSED_THRESHOLD}")

        rw = ResultsWrapper()
        start = perf_counter()
        for axis in axes:
            aw = AttemptWrapper()
            aw.type, aw.accuracy, aw.max_missed = "accel", accel_accu, self.MAX_MISSED_THRESHOLD
            aw.margin, aw.min, aw.max, aw.scv = margin, accel_min, accel_max, scv
            self.init_axis(aw, axis)
            rw.vals[aw.axis] = self.accel_binary_search(aw, samples)
        
        rw.duration = perf_counter() - start
        rw.name = "acceleration"
        
        respond = f"AUTO SPEED search found maximum acceleration after {rw.duration:.2f}s\n"
        for axis in self.valid_axes:
            if rw.vals.get(axis): respond += f"| {axis.replace('_', ' ').upper()} max: {rw.vals[axis]:.0f}\n"
        self.gcode.respond_info(respond)

        original_vals = dict(rw.vals)
        rw.derate(derate)
        
        respond = f"Recommended values (derated by {derate*100}%):\n"
        for axis, max_val in original_vals.items():
            if rw.vals.get(axis):
                respond += f"| {axis.replace('_', ' ').upper()} max: {rw.vals[axis]:.0f} (from {max_val:.0f})\n"
        respond += f"Recommended safe acceleration: {rw.vals['rec']:.0f}\n"
        self.gcode.respond_info(respond)
        
        self.run_safe_validation(rw.vals['rec'], validation_iters)
        return rw

    def accel_binary_search(self, aw: AttemptWrapper, samples_per_type: int) -> float:
        aw.time_start = perf_counter()
        m_min, m_max = aw.min, aw.max
        while (m_max - m_min) > (m_min * aw.accuracy):
            test_accel = (m_min + m_max) / 2.0
            if test_accel <= m_min or test_accel >= m_max: break
            self.gcode.respond_info(f"\n--- Testing accel value: {test_accel:.0f} ---")
            passed = self._run_gauntlet_for_accel(test_accel, aw, samples_per_type)
            if passed:
                self.gcode.respond_info(f"VALUE {test_accel:.0f} PASSED all tests.")
                m_min = test_accel
            else:
                self.gcode.respond_info(f"VALUE {test_accel:.0f} FAILED. This is the new ceiling.")
                m_max = test_accel
        aw.time_total = perf_counter() - aw.time_start
        self.gcode.respond_info(f"\nBinary search for axis {aw.axis} complete in {aw.time_total:.2f}s.")
        self.gcode.respond_info(f"Highest passing acceleration: {m_min:.0f}")
        return m_min

    def _run_gauntlet_for_accel(self, test_accel: float, aw: AttemptWrapper, samples_per_type: int) -> bool:
        last_known_steps, _ = self._prehome(aw.move.home)
        
        self.gcode.respond_info(f"--- Running {samples_per_type}x2 Accel-Focused Tests (Short & Sharp) ---")
        dist_a_theoretical = calculate_accel_focused_dist(self.accel_test_velocity, test_accel)
        dist_a = max(dist_a_theoretical, self.MIN_SHORT_MOVE_DISTANCE)
        aw.move.Calc(self.axis_limits, dist_a)
        for i in range(samples_per_type):
            self.gcode.respond_info(f"Sample {i*2+1}/{samples_per_type*2} (Forward)...")
            valid, current_steps = self._run_single_test_cycle(last_known_steps, aw, test_accel, forward=True)
            if not valid: return False
            last_known_steps = current_steps
            
            self.gcode.respond_info(f"Sample {i*2+2}/{samples_per_type*2} (Reverse)...")
            valid, current_steps = self._run_single_test_cycle(last_known_steps, aw, test_accel, forward=False)
            if not valid: return False
            last_known_steps = current_steps

        self.gcode.respond_info(f"--- Running {samples_per_type}x2 Velo-Plateau Tests (Long & Sustained) ---")
        dist_b = aw.move.max_safe_dist
        aw.move.Calc(self.axis_limits, dist_b)
        for i in range(samples_per_type):
            self.gcode.respond_info(f"Sample {i*2+1}/{samples_per_type*2} (Forward)...")
            valid, current_steps = self._run_single_test_cycle(last_known_steps, aw, test_accel, forward=True)
            if not valid: return False
            last_known_steps = current_steps
            
            self.gcode.respond_info(f"Sample {i*2+2}/{samples_per_type*2} (Reverse)...")
            valid, current_steps = self._run_single_test_cycle(last_known_steps, aw, test_accel, forward=False)
            if not valid: return False
            last_known_steps = current_steps
            
        return True

    def _run_single_test_cycle(self, start_steps, aw: AttemptWrapper, current_accel: float, forward: bool = True):
        self._set_velocity(self.default_velocity, self.default_accel, self.default_scv, self.default_accel_control_val)
        self._move(aw.move.center, self.default_velocity)
        self.toolhead.wait_moves()
        
        self._set_velocity(self.accel_test_velocity, current_accel, aw.scv, 0.0)
        if forward:
            self._move(aw.move.corner_a, self.accel_test_velocity)
            self._move(aw.move.corner_b, self.accel_test_velocity)
        else:
            self._move(aw.move.corner_b, self.accel_test_velocity)
            self._move(aw.move.corner_a, self.accel_test_velocity)
        self.toolhead.wait_moves()
        
        valid, current_steps, _, _ = self._posttest(start_steps, aw.max_missed, aw.move.home)
        return valid, current_steps

    def run_safe_validation(self, recommended_accel: float, iterations: int):
        passes, failures = 0, 0
        aw = AttemptWrapper()
        aw.max_missed = self.MAX_MISSED_THRESHOLD
        aw.scv = self.scv
        aw.home = [True, True, False]

        min_travel = min(self.axis_limits["x"]["dist"], self.axis_limits["y"]["dist"])
        max_safe_size = (min_travel / 3.0) - (10.0 / 1.5)

        if self.validation_pattern_size is None:
            size = max_safe_size
            self.gcode.respond_info(f"AUTO_SPEED: No validation pattern size configured. Automatically using max safe size: {size:.2f}mm")
        else:
            size = self.validation_pattern_size
            if size > max_safe_size:
                self.gcode.respond_info(f"ERROR: Configured validation_pattern_size ({size}mm) is unsafe for this printer.")
                self.gcode.respond_info(f"The maximum safe size is {max_safe_size:.2f}mm. Skipping validation.")
                return

        self.gcode.respond_info(f"--- Automated 'Chaos Star' Validation ({size:.1f}mm pattern) ---")
        self.gcode.respond_info(f"Performing {iterations} full iterations of the chaotic pattern.")

        center_x, center_y = self.axis_limits["x"]["center"], self.axis_limits["y"]["center"]
        half_size = size / 2.0
        c1 = [center_x - half_size, center_y - half_size, None]
        c2 = [center_x + half_size, center_y - half_size, None]
        c3 = [center_x + half_size, center_y + half_size, None]
        c4 = [center_x - half_size, center_y + half_size, None]
        m1 = [center_x - half_size, center_y, None]
        m2 = [center_x, center_y - half_size, None]
        m3 = [center_x + half_size, center_y, None]
        m4 = [center_x, center_y + half_size, None]
        ctr = [center_x, center_y, None]

        for i in range(iterations):
            self.gcode.respond_info(f"Validation Iteration {i+1}/{iterations}...")
            start_steps, _ = self._prehome(aw.home)
            
            self._set_velocity(self.default_velocity, self.default_accel, self.default_scv, self.default_accel_control_val)
            self._move(ctr, self.default_velocity)
            self.toolhead.wait_moves()
            
            self._set_velocity(self.accel_test_velocity, recommended_accel, aw.scv, 0.0)
            
            self._move(c1, self.accel_test_velocity); self._move(c2, self.accel_test_velocity); self._move(c3, self.accel_test_velocity); self._move(c4, self.accel_test_velocity); self._move(c1, self.accel_test_velocity)
            self._move(c3, self.accel_test_velocity); self._move(c1, self.accel_test_velocity); self._move(c2, self.accel_test_velocity); self._move(c4, self.accel_test_velocity); self._move(c3, self.accel_test_velocity)
            self._move(m1, self.accel_test_velocity); self._move(m3, self.accel_test_velocity); self._move(m2, self.accel_test_velocity); self._move(m4, self.accel_test_velocity); self._move(m1, self.accel_test_velocity)
            self._move(c2, self.accel_test_velocity)
            self.toolhead.wait_moves()

            valid, _, _, _ = self._posttest(start_steps, aw.max_missed, aw.home)
            if valid: passes += 1
            else: failures += 1
        
        self.gcode.respond_info("\n" + "="*40)
        self.gcode.respond_info("--- Automated Validation Complete ---")
        self.gcode.respond_info(f"Result: {passes} passes, {failures} failures out of {iterations} runs.")
        if failures == 0: self.gcode.respond_info("CONFIRMATION PASSED: Recommended acceleration is highly reliable.")
        else: self.gcode.respond_info("CONFIRMATION WARNING: Failures detected. Consider a lower value or increasing 'derate'.")
        self.gcode.respond_info("="*40)

    def handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.default_velocity = self.toolhead.max_velocity
        self.default_accel = self.toolhead.max_accel
        self.default_scv = self.toolhead.square_corner_velocity
        
        if hasattr(self.toolhead, 'min_cruise_ratio'):
            self.use_cruise_ratio = True
            self.default_accel_control_val = self.toolhead.min_cruise_ratio
            self.gcode.respond_info(f"AutoSpeed: Detected modern Klipper. Using 'min_cruise_ratio'.")
        else:
            self.use_cruise_ratio = False
            self.default_accel_control_val = self.toolhead.max_accel_to_decel
            self.gcode.respond_info(f"AutoSpeed: Detected older Klipper. Falling back to 'max_accel_to_decel'.")

        if self.printer.lookup_object("screw_tilt_adjust", None) is not None: self.level = "STA"
        elif self.printer.lookup_object("z_tilt", None) is not None: self.level = "ZT"
        elif self.printer.lookup_object("quad_gantry_level", None) is not None: self.level = "QGL"
        else: self.level = None

    def handle_home_rails_end(self, homing_state, rails):
        if not len(self.steppers.keys()) == 3:
            for rail in rails:
                pos_min, pos_max = rail.get_range()
                for stepper in rail.get_steppers():
                    name = stepper._name
                    if name in ["stepper_x", "stepper_y", "stepper_z"]:
                        config = self.printer.lookup_object('configfile').status_raw_config[name]
                        microsteps = int(config["microsteps"])
                        homing_retract_dist = float(config.get("homing_retract_dist", 5))
                        second_homing_speed = float(config.get("second_homing_speed", 5))
                        self.steppers[name[-1]] = [pos_min, pos_max, microsteps, homing_retract_dist, second_homing_speed]
            if self.steppers.get("x"): self.axis_limits["x"] = {"min": self.steppers["x"][0], "max": self.steppers["x"][1], "center": (self.steppers["x"][0] + self.steppers["x"][1]) / 2, "dist": self.steppers["x"][1] - self.steppers["x"][0], "home": self.gcode_move.homing_position[0]}
            if self.steppers.get("y"): self.axis_limits["y"] = {"min": self.steppers["y"][0], "max": self.steppers["y"][1], "center": (self.steppers["y"][0] + self.steppers["y"][1]) / 2, "dist": self.steppers["y"][1] - self.steppers["y"][0], "home": self.gcode_move.homing_position[1]}
            if self.steppers.get("z"): self.axis_limits["z"] = {"min": self.steppers["z"][0], "max": self.steppers["z"][1], "center": (self.steppers["z"][0] + self.steppers["z"][1]) / 2, "dist": self.steppers["z"][1] - self.steppers["z"][0], "home": self.gcode_move.homing_position[2]}

    cmd_AUTO_SPEED_VELOCITY_help = ("(Legacy) Finds max velocity using the original method.")
    def cmd_AUTO_SPEED_VELOCITY(self, gcmd):
        if not len(self.steppers.keys()) == 3: raise gcmd.error(f"Printer must be homed first!")
        axes = self._parse_axis(gcmd.get("AXIS", self._axis_to_str(self.axes)))
        margin = gcmd.get_float("MARGIN", self.margin, above=0.0)
        derate = gcmd.get_float('DERATE', self.derate, above=0.0, below=1.0)
        max_missed = gcmd.get_float('MAX_MISSED', self.max_missed_original, above=0.0)
        veloc_min = gcmd.get_float('VELOCITY_MIN', self.veloc_min, above=1.0)
        veloc_max = gcmd.get_float('VELOCITY_MAX', self.veloc_max, above=veloc_min)
        veloc_accu = gcmd.get_float('VELOCITY_ACCU', self.veloc_accu, above=0.0, below=1.0)
        accel = gcmd.get_float('ACCEL', 1.0, above=1.0)
        scv = gcmd.get_float('SCV', self.scv, above=1.0)
        self.gcode.respond_info(f"AUTO SPEED finding maximum velocity on {axes}")
        rw = ResultsWrapper()
        start = perf_counter()
        for axis in axes:
            aw = AttemptWrapper()
            aw.type, aw.accuracy, aw.max_missed = "velocity", veloc_accu, max_missed
            aw.margin, aw.min, aw.max, aw.accel, aw.scv = margin, veloc_min, veloc_max, accel, scv
            self.init_axis(aw, axis)
            rw.vals[aw.axis] = self.binary_search(aw)
        rw.duration = perf_counter() - start
        rw.name = "velocity"
        respond = f"AUTO SPEED found maximum velocity after {rw.duration:.2f}s\n"
        for axis in self.valid_axes:
            if rw.vals.get(axis): respond += f"| {axis.replace('_', ' ').upper()} max: {rw.vals[axis]:.0f}\n"
        respond += "\n"
        rw.derate(derate)
        respond += f"Recommended values\n"
        for axis in self.valid_axes:
            if rw.vals.get('max_' + axis): respond += f"| {axis.replace('_', ' ').upper()} max: {rw.vals[axis]:.0f}\n"
        respond += f"Recommended velocity: {rw.vals['rec']:.0f}\n"
        self.gcode.respond_info(respond)
        return rw

    cmd_AUTO_SPEED_VALIDATE_help = ("(Legacy) Validates settings using the original unsafe pattern.")
    def cmd_AUTO_SPEED_VALIDATE(self, gcmd):
        if not len(self.steppers.keys()) == 3: raise gcmd.error(f"Printer must be homed first!")
        max_missed = gcmd.get_float('MAX_MISSED', self.max_missed_original, above=0.0)
        margin = gcmd.get_float('VALIDATE_MARGIN', default=self.margin, above=0.0)
        small_margin = gcmd.get_float('VALIDATE_INNER_MARGIN', default=self.validate_inner_margin, above=0.0)
        iterations = gcmd.get_int('VALIDATE_ITERATIONS', default=self.validate_iterations, minval=1)
        accel = gcmd.get_float('ACCEL', default=self.toolhead.max_accel, above=0.0)
        veloc = gcmd.get_float('VELOCITY', default=self.toolhead.max_velocity, above=0.0)
        scv = gcmd.get_float('SCV', default=self.toolhead.square_corner_velocity, above=1.0)
        respond = f"AUTO SPEED validating over {iterations} iterations\n"
        respond += f"Acceleration: {accel:.0f}\n"
        respond += f"Velocity: {veloc:.0f}\n"
        respond += f"SCV: {scv:.0f}"
        self.gcode.respond_info(respond)
        self._set_velocity(veloc, accel, scv)
        valid, duration, missed_x, missed_y = self._validate(veloc, iterations, margin, small_margin, max_missed)
        respond = f"AUTO SPEED validated results after {duration:.2f}s\n"
        respond += f"Valid: {valid}\n"
        respond += f"Missed X {missed_x:.2f}, Y {missed_y:.2f}"
        self.gcode.respond_info(respond)
        return valid

    cmd_AUTO_SPEED_GRAPH_help = ("(Legacy) Graphs accel vs. velocity using the original method.")
    def cmd_AUTO_SPEED_GRAPH(self, gcmd):
        import matplotlib.pyplot as plt
        if not len(self.steppers.keys()) == 3: raise gcmd.error(f"Printer must be homed first!")
        axes = self._parse_axis(gcmd.get("AXIS", self._axis_to_str(self.axes)))
        margin = gcmd.get_float("MARGIN", self.margin, above=0.0)
        derate = gcmd.get_float('DERATE', self.derate, above=0.0, below=1.0)
        max_missed = gcmd.get_float('MAX_MISSED', self.max_missed_original, above=0.0)
        scv = gcmd.get_float('SCV', default=self.toolhead.square_corner_velocity, above=1.0)
        veloc_min = gcmd.get_float('VELOCITY_MIN', 200.0, above=0.0)
        veloc_max = gcmd.get_float('VELOCITY_MAX', 700.0, above=veloc_min)
        veloc_div = gcmd.get_int('VELOCITY_DIV', 5, minval=0)
        accel_accu = gcmd.get_float('ACCEL_ACCU', 0.05, above=0.0, below=1.0)
        accel_min_slope = gcmd.get_int('ACCEL_MIN_SLOPE', 100, minval=0)
        accel_max_slope = gcmd.get_int('ACCEL_MAX_SLOPE', 1800, minval=accel_min_slope)
        veloc_step = (veloc_max - veloc_min) // (veloc_div - 1) if veloc_div > 1 else 0
        velocs = [round((v * veloc_step) + veloc_min) for v in range(veloc_div)]
        respond = f"AUTO SPEED graphing maximum accel from velocities on {axes}\n"
        respond += f"V_MIN: {veloc_min}, V_MAX: {veloc_max}, V_STEP: {veloc_step}\n"
        self.gcode.respond_info(respond)
        aw = AttemptWrapper()
        aw.type, aw.accuracy, aw.max_missed, aw.margin, aw.scv = "graph", accel_accu, max_missed, margin, scv
        for axis in axes:
            start = perf_counter()
            self.init_axis(aw, axis)
            accels, accel_mins, accel_maxs = [], [], []
            for veloc in velocs:
                self.gcode.respond_info(f"AUTO SPEED graph {aw.axis} - v{veloc}")
                aw.veloc = veloc
                aw.min = round(calculate_graph(veloc, accel_min_slope))
                aw.max = round(calculate_graph(veloc, accel_max_slope))
                accel_mins.append(aw.min)
                accel_maxs.append(aw.max)
                accels.append(self.binary_search(aw))
            plt.plot(velocs, accels, 'go-', label='measured')
            plt.plot(velocs, [a * derate for a in accels], 'g-', label='derated')
            plt.plot(velocs, accel_mins, 'b--', label='min')
            plt.plot(velocs, accel_maxs, 'r--', label='max')
            plt.legend(loc='upper right')
            plt.title(f"Max accel at velocity on {aw.axis} to {int(accel_accu*100)}% accuracy")
            plt.xlabel("Velocity")
            plt.ylabel("Acceleration")
            filepath = os.path.join(self.results_dir, f"AUTO_SPEED_GRAPH_{dt.datetime.now():%Y-%m-%d_%H:%M:%S}_{aw.axis}.png")
            self.gcode.respond_info(f"Velocs: {velocs}")
            self.gcode.respond_info(f"Accels: {accels}")
            self.gcode.respond_info(f"AUTO SPEED graph found max accel on {aw.axis} after {perf_counter() - start:.0f}s\nSaving graph to {filepath}")
            os.makedirs(self.results_dir, exist_ok=True)
            plt.savefig(filepath, bbox_inches='tight')
            plt.close()

    def _prepare(self, gcmd):
        if not len(self.steppers.keys()) == 3: raise gcmd.error(f"Printer must be homed first!")
        start = perf_counter()
        self._level(gcmd)
        self._set_velocity(self.default_velocity, self.default_accel, self.default_scv, self.default_accel_control_val)
        self._move([self.axis_limits["x"]["center"], self.axis_limits["y"]["center"], self.axis_limits["z"]["center"]], self.default_velocity)
        self.gcode.run_script_from_command("M400")
        self.gcode.respond_info("Toolhead centered and settled. Beginning tests.")
        self._variance(gcmd)
        return perf_counter() - start

    def _level(self, gcmd):
        level = gcmd.get_int('LEVEL', 1, minval=0, maxval=1)
        if level == 0 or self.level is None: return
        lookup, name = None, None
        if self.level == "STA": lookup, name = "screw_tilt_adjust", "SCREWS_TILT_CALCULATE"
        elif self.level == "ZT": lookup, name = "z_tilt", "Z_TILT_ADJUST"
        elif self.level == "QGL": lookup, name = "quad_gantry_level", "QUAD_GANTRY_LEVEL"
        else: raise gcmd.error(f"Unknown leveling method '{self.level}'.")
        lm = self.printer.lookup_object(lookup)
        if not lm.z_status.applied:
            self.gcode.respond_info(f"AUTO SPEED leveling with {name}...")
            self.gcode._process_commands([name], False)
            if not lm.z_status.applied: raise gcmd.error(f"Failed to level printer!")

    def _variance(self, gcmd):
        if gcmd.get_int('VARIANCE', 1, minval=0, maxval=1) == 0: return
        max_missed = gcmd.get_float('MAX_MISSED', self.max_missed_original, above=0.0)
        endstop_samples = gcmd.get_int('ENDSTOP_SAMPLES', self.endstop_samples, minval=2)
        if gcmd.get_int("SETTLING_HOME", default=self.settling_home, minval=0, maxval=1):
            self.toolhead.wait_moves()
            self._home(True, True, False)
        axes = self._parse_axis(gcmd.get("AXIS", self._axis_to_str(self.axes)))
        check_x = 'x' in axes if self.isolate_xy else True
        check_y = 'y' in axes if self.isolate_xy else True
        endstops = self._endstop_variance(endstop_samples, x=check_x, y=check_y)
        x_max = max(endstops["x"]) if check_x and endstops["x"] else 0
        y_max = max(endstops["y"]) if check_y and endstops["y"] else 0
        self.gcode.respond_info(f"AUTO SPEED endstop variance:\nMissed X:{x_max:.2f} steps, Y:{y_max:.2f} steps")
        if x_max >= max_missed or y_max >= max_missed:
            raise gcmd.error(f"Please increase MAX_MISSED (currently {max_missed}), or tune your steppers/homing macro.")

    def _parse_axis(self, raw_axes):
        return [axis for axis in raw_axes.lower().replace(" ", "").split(',') if axis in self.valid_axes]

    def _axis_to_str(self, raw_axes):
        return ",".join(raw_axes)

    def init_axis(self, aw: AttemptWrapper, axis):
        aw.axis = axis
        if axis == "diag_x": aw.move = MoveDiagX()
        elif axis == "diag_y": aw.move = MoveDiagY()
        elif axis == "x": aw.move = MoveX()
        elif axis == "y": aw.move = MoveY()
        elif axis == "z": aw.move = MoveZ()
        aw.move.Init(self.axis_limits, aw.margin, self.isolate_xy)

    def binary_search(self, aw: AttemptWrapper):
        aw.time_start = perf_counter()
        m_min, m_max = aw.min, aw.max
        m_var = m_min + (m_max - m_min) // 3
        if aw.veloc == 0.0: aw.veloc = 1.0
        if aw.accel == 0.0: aw.accel = 1.0
        from .funcs import calculate_accel, calculate_velocity
        if aw.type in ("accel", "graph"):
            m_stat, o_veloc = aw.veloc, aw.veloc
            if o_veloc == 1.0: aw.accel = calculate_accel(aw.veloc, aw.move.max_dist)
            aw.move.Calc(self.axis_limits, m_stat, m_var, aw.margin)
        elif aw.type == "velocity":
            m_stat, o_accel = aw.accel, aw.accel
            if o_accel == 1.0: aw.veloc = calculate_velocity(aw.accel, aw.move.max_dist)
            aw.move.Calc(self.axis_limits, m_var, m_stat, aw.margin)
        
        measured_val = None
        aw.tries = 0
        aw.home_steps, aw.move_time_prehome = self._prehome(aw.move.home)
        while True:
            aw.tries += 1
            if aw.type in ("accel", "graph"):
                if o_veloc == 1.0: m_stat = aw.veloc = calculate_velocity(m_var, aw.move.dist) / 2.5
                aw.accel = m_var
                aw.move.Calc(self.axis_limits, m_stat, m_var, aw.margin)
            elif aw.type == "velocity":
                if o_accel == 1.0: m_stat = aw.accel = calculate_accel(m_var, aw.move.dist) * 2.5
                aw.veloc = m_var
                aw.move.Calc(self.axis_limits, m_var, m_stat, aw.margin)
            
            valid = self._attempt(aw)
            veloc = m_var if aw.type == "velocity" else m_stat
            accel = m_var if aw.type in ("accel", "graph") else m_stat
            
            respond = f"AUTO SPEED {aw.type} on {aw.axis} try {aw.tries} ({aw.time_last:.2f}s)\n"
            respond += f"Moved {aw.move_dist - aw.margin:.2f}mm at a{accel:.0f}/v{veloc:.0f} after {aw.move_time_prehome:.2f}/{aw.move_time:.2f}/{aw.move_time_posthome:.2f}s\n"
            respond += f"Missed"
            if aw.move.home[0]: respond += f" X {aw.missed['x']:.2f},"
            if aw.move.home[1]: respond += f" Y {aw.missed['y']:.2f},"
            if aw.move.home[2]: respond += f" Z {aw.missed['z']:.2f},"
            self.gcode.respond_info(respond[:-1])
            
            if measured_val is not None:
                if m_var * (1 + aw.accuracy) > m_max or m_var * (1 - aw.accuracy) < m_min:
                    break
            measured_val = m_var
            if valid: m_min = m_var
            else: m_max = m_var
            m_var = (m_min + m_max) // 2
        aw.time_total = perf_counter() - aw.time_start
        return m_var

    def _attempt(self, aw: AttemptWrapper):
        timeAttempt = perf_counter()
        self._set_velocity(self.default_velocity, self.default_accel, self.default_scv, self.default_accel_control_val)
        self._move([aw.move.pos["x"][0], aw.move.pos["y"][0], aw.move.pos["z"][0]], self.default_velocity)
        self.toolhead.wait_moves()
        self._set_velocity(aw.veloc, aw.accel, aw.scv)
        timeMove = perf_counter()
        self._move([aw.move.pos["x"][1], aw.move.pos["y"][1], aw.move.pos["z"][1]], aw.veloc)
        self.toolhead.wait_moves()
        aw.move_time = perf_counter() - timeMove
        aw.move_dist = aw.move.dist
        valid, aw.home_steps, aw.missed, aw.move_time_posthome = self._posttest(aw.home_steps, aw.max_missed, aw.move.home)
        aw.time_last = perf_counter() - timeAttempt
        return valid

    def _validate(self, speed, iterations, margin, small_margin, max_missed):
        pos = {"x": {"min": self.axis_limits["x"]["min"] + margin, "max": self.axis_limits["x"]["max"] - margin, "center_min": self.axis_limits["x"]["center"] - (small_margin / 2), "center_max": self.axis_limits["x"]["center"] + (small_margin / 2)}, "y": {"min": self.axis_limits["y"]["min"] + margin, "max": self.axis_limits["y"]["max"] - margin, "center_min": self.axis_limits["y"]["center"] - (small_margin / 2), "center_max": self.axis_limits["y"]["center"] + (small_margin / 2)}}
        self.toolhead.wait_moves()
        self._home(True, True, False)
        start_steps = self._get_steps()
        start = perf_counter()
        for _ in range(iterations):
            self._move([pos["x"]["min"], pos["y"]["min"], None], speed)
            self._move([pos["x"]["max"], pos["y"]["max"], None], speed)
            self._move([pos["x"]["min"], pos["y"]["min"], None], speed)
            self._move([pos["x"]["max"], pos["y"]["min"], None], speed)
            self._move([pos["x"]["min"], pos["y"]["max"], None], speed)
            self._move([pos["x"]["max"], pos["y"]["min"], None], speed)
            self._move([pos["x"]["min"], pos["y"]["min"], None], speed)
            self._move([pos["x"]["min"], pos["y"]["max"], None], speed)
            self._move([pos["x"]["max"], pos["y"]["max"], None], speed)
            self._move([pos["x"]["max"], pos["y"]["min"], None], speed)
            self._move([pos["x"]["center_min"], pos["y"]["center_min"], None], speed)
            self._move([pos["x"]["center_max"], pos["y"]["center_max"], None], speed)
            self._move([pos["x"]["center_min"], pos["y"]["center_min"], None], speed)
            self._move([pos["x"]["center_max"], pos["y"]["center_min"], None], speed)
            self._move([pos["x"]["center_min"], pos["y"]["center_max"], None], speed)
            self._move([pos["x"]["center_max"], pos["y"]["center_min"], None], speed)
            self._move([pos["x"]["center_min"], pos["y"]["center_min"], None], speed)
            self._move([pos["x"]["center_min"], pos["y"]["center_max"], None], speed)
            self._move([pos["x"]["center_max"], pos["y"]["center_max"], None], speed)
            self._move([pos["x"]["center_max"], pos["y"]["center_min"], None], speed)
        self.toolhead.wait_moves()
        duration = perf_counter() - start
        self._home(True, True, False)
        stop_steps = self._get_steps()
        step_dif = {"x": abs(start_steps["x"] - stop_steps["x"]), "y": abs(start_steps["y"] - stop_steps["y"])}
        missed_x = step_dif['x'] / self.steppers['x'][2]
        missed_y = step_dif['y'] / self.steppers['y'][2]
        valid = not (missed_x > max_missed or missed_y > max_missed)
        return valid, duration, missed_x, missed_y

    def _endstop_variance(self, samples: int, x=True, y=True):
        variance = {"x": [], "y": [], "steps": {"x": None, "y": None}}
        for _ in range(samples):
            self.toolhead.wait_moves()
            self._home(x, y, False)
            steps = self._get_steps()
            if x and variance["steps"]["x"] is not None: variance["x"].append(abs(variance["steps"]["x"] - steps["x"]) / self.steppers['x'][2])
            if y and variance["steps"]["y"] is not None: variance["y"].append(abs(variance["steps"]["y"] - steps["y"]) / self.steppers['y'][2])
            if x: variance["steps"]["x"] = steps["x"]
            if y: variance["steps"]["y"] = steps["y"]
        return variance

    def _move(self, coord, speed):
        self.toolhead.manual_move(coord, speed)

    def _home(self, x=True, y=True, z=True):
        self._set_velocity(self.default_velocity, self.default_accel, self.default_scv, self.default_accel_control_val)
        command = ["G28"]
        if x: command[-1] += " X0"
        if y: command[-1] += " Y0"
        if z: command[-1] += " Z0"
        self.gcode._process_commands(command, False)
        self.toolhead.wait_moves()

    def _get_steps(self):
        steppers = self.toolhead.get_kinematics().get_steppers()
        return {s.get_name()[-1]: s.get_mcu_position() for s in steppers if s.get_name() in ["stepper_x", "stepper_y", "stepper_z"]}

    def _prehome(self, home: list):
        self.toolhead.wait_moves()
        dur = perf_counter()
        self._home(home[0], home[1], home[2])
        dur = perf_counter() - dur
        return self._get_steps(), dur

    def _posttest(self, start_steps, max_missed, home: list):
        self.toolhead.wait_moves()
        dur = perf_counter()
        self._home(home[0], home[1], home[2])
        dur = perf_counter() - dur
        valid = True
        stop_steps = self._get_steps()
        missed = {}
        if home[0]:
            missed["x"] = abs(start_steps["x"] - stop_steps["x"]) / self.steppers['x'][2]
            if missed["x"] > max_missed: valid = False
        if home[1]:
            missed["y"] = abs(start_steps["y"] - stop_steps["y"]) / self.steppers['y'][2]
            if missed["y"] > max_missed: valid = False
        if home[2]:
            missed["z"] = abs(start_steps["z"] - stop_steps["z"]) / self.steppers['z'][2]
            if missed["z"] > max_missed: valid = False
        return valid, stop_steps, missed, dur

    def _set_velocity(self, velocity: float, accel: float, scv: float, accel_control_val: float = None):
        cmd = f"SET_VELOCITY_LIMIT VELOCITY={velocity} ACCEL={accel} SQUARE_CORNER_VELOCITY={scv}"
        if accel_control_val is not None:
            if self.use_cruise_ratio:
                cmd += f" MINIMUM_CRUISE_RATIO={accel_control_val}"
            else:
                cmd += f" ACCEL_TO_DECEL={accel_control_val}"
        self.gcode.run_script_from_command(cmd)

    def cmd_X_ENDSTOP_ACCURACY(self, gcmd):
        if not len(self.steppers.keys()) == 3: raise gcmd.error(f"Printer must be homed first!")
        sample_count = gcmd.get_int("SAMPLES", 10, minval=1)
        second_homing_speed = self.steppers['x'][4]
        homing_retract_dist = self.steppers['x'][3]
        toolhead = self.printer.lookup_object('toolhead')
        pos = toolhead.get_position()
        gcmd.respond_info(f"X_ENDSTOP_ACCURACY at X:{pos[0]:.3f} (samples={sample_count})\nSecond Homing Speed: {second_homing_speed:.2f} mm/s\nHoming Retract Distance: {homing_retract_dist:.2f} mm")
        positions = []
        for _ in range(sample_count):
            self._home(True, False, False)
            pos = toolhead.get_position()
            positions.append(pos[0])
            toolhead.manual_move([pos[0] - homing_retract_dist, None, None], speed=second_homing_speed)
        max_v, min_v, avg_v = max(positions), min(positions), sum(positions) / len(positions)
        range_v = max_v - min_v
        sigma = (sum([(x - avg_v) ** 2 for x in positions]) / len(positions)) ** 0.5
        gcmd.respond_info(f"X endstop accuracy results: maximum {max_v:.6f}, minimum {min_v:.6f}, range {range_v:.6f}, average {avg_v:.6f}, standard deviation {sigma:.6f}")

    def cmd_Y_ENDSTOP_ACCURACY(self, gcmd):
        if not len(self.steppers.keys()) == 3: raise gcmd.error(f"Printer must be homed first!")
        sample_count = gcmd.get_int("SAMPLES", 10, minval=1)
        second_homing_speed = self.steppers['y'][4]
        homing_retract_dist = self.steppers['y'][3]
        toolhead = self.printer.lookup_object('toolhead')
        pos = toolhead.get_position()
        gcmd.respond_info(f"Y_ENDSTOP_ACCURACY at Y:{pos[1]:.3f} (samples={sample_count})\nSecond Homing Speed: {second_homing_speed:.2f} mm/s\nHoming Retract Distance: {homing_retract_dist:.2f} mm")
        positions = []
        for _ in range(sample_count):
            self._home(False, True, False)
            pos = toolhead.get_position()
            positions.append(pos[1])
            toolhead.manual_move([None, pos[1] - homing_retract_dist, None], speed=second_homing_speed)
        max_v, min_v, avg_v = max(positions), min(positions), sum(positions) / len(positions)
        range_v = max_v - min_v
        sigma = (sum([(y - avg_v) ** 2 for y in positions]) / len(positions)) ** 0.5
        gcmd.respond_info(f"Y endstop accuracy results: maximum {max_v:.6f}, minimum {min_v:.6f}, range {range_v:.6f}, average {avg_v:.6f}, standard deviation {sigma:.6f}")

    def cmd_Z_ENDSTOP_ACCURACY(self, gcmd):
        if not len(self.steppers.keys()) == 3: raise gcmd.error(f"Printer must be homed first!")
        sample_count = gcmd.get_int("SAMPLES", 10, minval=1)
        second_homing_speed = self.steppers['z'][4]
        homing_retract_dist = self.steppers['z'][3]
        toolhead = self.printer.lookup_object('toolhead')
        pos = toolhead.get_position()
        gcmd.respond_info(f"Z_ENDSTOP_ACCURACY at Z:{pos[2]:.3f} (samples={sample_count})\nSecond Homing Speed: {second_homing_speed:.2f} mm/s\nHoming Retract Distance: {homing_retract_dist:.2f} mm")
        positions = []
        for _ in range(sample_count):
            self._home(False, False, True)
            pos = toolhead.get_position()
            positions.append(pos[2])
            toolhead.manual_move([None, None, pos[2] + homing_retract_dist], speed=second_homing_speed)
        max_v, min_v, avg_v = max(positions), min(positions), sum(positions) / len(positions)
        range_v = max_v - min_v
        sigma = (sum([(z - avg_v) ** 2 for z in positions]) / len(positions)) ** 0.5
        gcmd.respond_info(f"Z endstop accuracy results: maximum {max_v:.6f}, minimum {min_v:.6f}, range {range_v:.6f}, average {avg_v:.6f}, standard deviation {sigma:.6f}")