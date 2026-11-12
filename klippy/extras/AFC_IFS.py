# FlashForge AD5X IFS unit for Armored Turtle AFC
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import json
import os
import traceback

from configparser import Error as config_error

try: from extras.AFC_utils import ERROR_STR
except:
    trace=traceback.format_exc()
    err_str = f"Error when trying to import AFC_utils.ERROR_STR\n{trace}"
    raise config_error(err_str)

try: from extras.AFC_lane import MoveDirection, AFCMoveWarning, AFCLaneState, SpeedMode
except: raise config_error(ERROR_STR.format(import_lib="AFC_lane", trace=traceback.format_exc()))

try: from extras.AFC_HTLF import AFC_HTLF
except: raise config_error(ERROR_STR.format(import_lib="AFC_HTLF", trace=traceback.format_exc()))

SILK_SAMPLE_TIME  = 0.001
SILK_SAMPLE_COUNT = 20
SILK_REPORT_TIME  = 0.100
SPEED_REPORT_TIME = 0.038   # stock: an average every 18 to 80 ms (estimate), the 20 ms tick reuses the last one
SPEED_TICK        = 0.020
# stock getFilamentTypeUnloadSpeed, mm/min, other types 600
STOCK_UNLOAD = {"PLA": 600., "PETG": 300., "TPU": 200., "SILK": 300.}
# stock load wizard: nozzle temperature by type (updateFilamentTemp), first flush speed mm/min and fans 0-255
STOCK_TEMP   = {"PLA": 220., "PLA-CF": 220., "SILK": 230., "PETG": 230., "TPU": 250., "PA": 270., "PC-ABS": 270.,
                "PET-CF": 280., "PPS-CF": 280., "PC": 280., "PA-CF": 290., "PAHT-CF": 305.}
FLUSH_SPEED  = {"PLA": 300., "PETG": 300., "TPU": 100., "SILK": 150.}
FLUSH_FAN    = {"PLA": 0, "PETG": 51, "TPU": 51, "SILK": 51}
AFTER_FAN    = {"PLA": 125, "PETG": 102, "TPU": 102, "SILK": 102}
# stock SysTick odometer latch (20 ms): a change of 15 counts against the first sample of a 50 sample
# window means the filament moves; 50 quiet samples clear the flag. The counter restarts with each feed
class _Odometer:
    def __init__(self):
        self.window = []
        self.flag = False
        self.count = 0

    def feed(self, counts):
        if not self.window:
            self.window.append(counts)
            return
        if abs(counts - self.window[0]) >= 15:
            self.flag = True
            self.count += 1
            self.window = []
            return
        self.window.append(counts)
        if len(self.window) >= 50:
            self.flag = False
            self.window = []

# the web UI draws an on/off switch for every filament sensor entry, this one is the end of print unload setting
class _EndUnloadSwitch:
    def __init__(self, unit):
        self.unit = unit

    def get_status(self, eventtime):
        return {"filament_detected": False, "enabled": self.unit.end_unload}


class AFC_IFS(AFC_HTLF):
    def __init__(self, config):
        super().__init__(config)
        self.slot_angle         = config.getfloat("slot_angle", 90.)
        self.home_backoff       = config.getfloat("home_backoff", 90.)
        self.home_search        = config.getfloat("home_search", 360.)
        self.neutral_angle      = config.getfloat("neutral_angle", 45.)
        self.rehome_on_select   = config.getboolean("rehome_on_select", True)
        self.silk_present_low   = config.getfloat("silk_present_low", 900. / 4095.)
        self.silk_present_high  = config.getfloat("silk_present_high", 3100. / 4095.)
        self.silk_empty_low     = config.getfloat("silk_empty_low", 1001. / 4095.)
        self.silk_empty_high    = config.getfloat("silk_empty_high", 2999. / 4095.)
        self.silk_pins          = [p.strip() for p in config.get("silk_pins").split(",")]
        self.silk_state         = [None] * len(self.silk_pins)
        self.silk_raw           = [0] * len(self.silk_pins)
        self.silk_default       = {"present_low": self.silk_present_low * 4095.,
                                   "empty_low": self.silk_empty_low * 4095.,
                                   "empty_high": self.silk_empty_high * 4095.,
                                   "present_high": self.silk_present_high * 4095.}
        self.silk_limits        = [dict(self.silk_default) for _ in self.silk_pins]
        self.silk_cal           = [{} for _ in self.silk_pins]
        # Fluidd looks the unit up as AFC_<type> <name> from the type in AFC's unit list
        self.printer.add_object("AFC_{} {}".format(self.type, self.name), self)
        self.silk_file          = config.get("silk_file", "/usr/data/openad5x/klipper/config/AFC/ifs_silk.json")
        self._silk_load()
        self.speed_pins         = [p.strip() for p in config.get("speed_pins", "").split(",") if p.strip()]
        self.odo                = [_Odometer() for _ in self.speed_pins]
        self.speed_latch        = [0] * len(self.speed_pins)
        # stock OdometerEnable: pause after 200 s of extrusion with a still roller
        self.odometer_enable    = config.getboolean("odometer_enable", False)
        self.odo_count          = [0] * len(self.speed_pins)
        self.odo_last_e         = 0.
        self.auto_load          = config.getboolean("auto_load", True)
        self.end_unload         = config.getboolean("end_unload", True)
        self.options_file       = config.get("options_file", "/usr/data/openad5x/klipper/config/AFC/ifs_options.json")
        self._options_load()
        self.auto_load_max     = config.getfloat("auto_load_max", 650.)
        self.park_dist          = config.getfloat("park_dist", 100.)
        self.tool_sensor        = config.get("tool_sensor", "fill_sensor")
        # stock checkFeeding: feed until the toolhead sensor (30 s at 20 mm/s), back 5 mm, feed 500 mm
        self.load_feed          = config.getfloat("load_feed", 600.)
        self.load_backoff       = config.getfloat("load_backoff", 5.)
        self.load_retry_feed    = config.getfloat("load_retry_feed", 500.)
        self.auto_load_busy     = False
        self.selector_pos       = None
        # purge chute, stock firmwareExe printClearGoPos / clear_brush positions
        self.chute_x            = config.getfloat("chute_x", 52.5)
        self.chute_y            = config.getfloat("chute_y", 230.5)
        self.chute_y_out        = config.getfloat("chute_y_out", 220.)
        self.brush_x_min        = config.getfloat("brush_x_min", 65.)
        self.brush_x_max        = config.getfloat("brush_x_max", 73.)
        self.brush_x_out        = config.getfloat("brush_x_out", 78.)
        self.brush_count        = config.getint("brush_count", 2)
        self.ifs_gcode          = self.printer.lookup_object("gcode")
        self.ifs_gcode.register_command("ADX_PARK", self.cmd_ADX_PARK, desc="Move into the purge chute like stock")
        self.ifs_gcode.register_command("ADX_POOP", self.cmd_ADX_POOP, desc="Purge in the chute like stock")
        self.ifs_gcode.register_command("ADX_BRUSH", self.cmd_ADX_BRUSH, desc="Stock brush wipe, ends out of the chute")
        self.cut_approach_y     = config.getfloat("cut_approach_y", 10.)
        # stock cutFilament "F11 C<lane> L60" with "G1 E-60", then "F11 C<lane> L70 S1200"
        self.unload_cut_dist    = config.getfloat("unload_cut_dist", 60.)
        self.unload_ifs_dist    = config.getfloat("unload_ifs_dist", 70.)
        self.last_index         = None
        self.ifs_gcode.register_command("ADX_SILK_STATUS", self.cmd_ADX_SILK_STATUS, desc="Show each lane's raw silk value and limits")
        self.ifs_gcode.register_command("ADX_SILK_CALIBRATE", self.cmd_ADX_SILK_CALIBRATE,
                                        desc="Calibrate a lane's silk limits: LANE=<1-4> STATE=empty|present")
        self.ifs_gcode.register_command("ADX_SILK_SET", self.cmd_ADX_SILK_SET,
                                        desc="Set a lane's silk limits: LANE=<1-4> PRESENT_LOW= EMPTY_LOW= EMPTY_HIGH= PRESENT_HIGH=")
        self.ifs_gcode.register_command("ADX_SILK_RESET", self.cmd_ADX_SILK_RESET,
                                        desc="Reset silk limits to stock: LANE=<1-4> or all")
        self.ifs_gcode.register_command("ADX_LOAD_HEAD", self.cmd_ADX_LOAD_HEAD,
                                        desc="Stock load wizard: heat, feed the lane to the head, purge: LANE=<1-4> TEMP=<C>")
        self.ifs_gcode.register_command("ADX_CUT", self.cmd_ADX_CUT, desc="Stock approach to the cutter, then AFC_CUT")
        self.ifs_gcode.register_command("ADX_END_UNLOAD", self.cmd_ADX_END_UNLOAD,
                                        desc="Unload the lane in the toolhead at the end of a print, if enabled")
        self.ifs_gcode.register_command("ADX_END_UNLOAD_SET", self.cmd_ADX_END_UNLOAD_SET,
                                        desc="Turn the end of print unload on or off: ENABLE=0|1, no value shows it")
        self.printer.add_object("filament_switch_sensor end_unload", _EndUnloadSwitch(self))
        self.ifs_gcode.register_mux_command("SET_FILAMENT_SENSOR", "SENSOR", "end_unload", self.cmd_ADX_END_UNLOAD_SET,
                                            desc="Turn the end of print unload on or off")

        ppins = self.printer.lookup_object("pins")
        for idx, pin in enumerate(self.speed_pins if self.odometer_enable else []):
            ppins.allow_multi_use_pin(pin)
            mcu_adc = ppins.setup_pin("adc", pin)
            mcu_adc.setup_adc_sample(SILK_SAMPLE_TIME, SILK_SAMPLE_COUNT)
            mcu_adc.setup_adc_callback(SPEED_REPORT_TIME,
                                       lambda t, v, i=idx: self.speed_latch.__setitem__(i, int(v * 4095.)))
        for idx, pin in enumerate(self.silk_pins):
            mcu_adc = ppins.setup_pin("adc", pin)
            mcu_adc.setup_adc_sample(SILK_SAMPLE_TIME, SILK_SAMPLE_COUNT)
            mcu_adc.setup_adc_callback(SILK_REPORT_TIME,
                                       lambda t, v, i=idx: self._silk_callback(i, t, v))

    def handle_connect(self):
        super().handle_connect()
        self.logo = '<span class=success--text>IFS Ready\n</span>'
        self.logo_error = '<span class=error--text>IFS Not Ready</span>\n'
        if self.afc.park_cmd == "AFC_PARK":
            self.afc.park_cmd = "ADX_PARK"
        if self.afc.poop_cmd == "AFC_POOP":
            self.afc.poop_cmd = "ADX_POOP"
        if self.afc.wipe_cmd == "AFC_BRUSH":
            self.afc.wipe_cmd = "ADX_BRUSH"
        if self.afc.tool_cut_cmd == "AFC_CUT":
            self.afc.tool_cut_cmd = "ADX_CUT"
        self.afc_restore_pos = self.afc.restore_pos
        self.afc.restore_pos = self._restore_pos
        self.afc_change_tool = self.afc.CHANGE_TOOL
        self.afc.CHANGE_TOOL = self._change_tool
        self.afc_poop_kick_wipe = self.afc.do_poop_kick_wipe
        self.afc.do_poop_kick_wipe = self._poop_kick_wipe
        self.poop_lane = None
        self.poop_extruder = None
        sensor =self.printer.lookup_object("filament_motion_sensor " + self.tool_sensor)
        self.drive_stepper_obj._add_endstop("tool_start", None, "tool_start", mcu_endstop=sensor.endstop)
        # stock: feed motor off while printing, the extruder pulls the lane through
        drive = self.drive_stepper_obj
        drive.sync_to_extruder = lambda *args, **kwargs: self._release(drive)
        self.afc.reactor.register_timer(self._odometer_tick, self.afc.reactor.NOW)
        self.afc.reactor.register_timer(self._odometer_sample, self.afc.reactor.NOW)
        self.afc.reactor.register_timer(self._gconf_check, self.afc.reactor.monotonic() + 5.)
        # stock: selector off on arrival; the feed stays on from F10 until F112 or the F11 stop
        toolhead = self.printer.lookup_object("toolhead")
        for name in ("move", "do_homing_move"):
            def wrapped(*a, _o=getattr(self.selector_stepper_obj, name), **k):
                try:
                    return _o(*a, **k)
                finally:
                    toolhead.wait_moves()
                    self._release(self.selector_stepper_obj)
            setattr(self.selector_stepper_obj, name, wrapped)

    # stock checkFeeding: one long feed that stops when the toolhead sensor sees filament
    def unit_load_lane(self, lane, extruder):
        self._chute_park()
        self._feed_started(lane)
        endstop = lane.get_toolhead_endstop()
        try:
            homed, dist, warn = lane.move_to(self.load_feed, SpeedMode.LONG, endstop=endstop)
            if not homed:
                lane.move(-self.load_backoff, lane.long_moves_speed, lane.long_moves_accel)
                self._release(self.drive_stepper_obj)
                homed, dist, warn = lane.move_to(self.load_retry_feed, SpeedMode.LONG, endstop=endstop)
        finally:
            self._release(self.drive_stepper_obj)
        if not homed:
            self.afc.error.handle_lane_failure(lane, "E{:04d}: channel {} feeding timeout".format(99 + lane.index, lane.index))
            return False
        lane.status = AFCLaneState.TOOL_LOADED
        return True

    # stock: the host polls F13 every 200 ms idle and about every second printing, and each reply reads GCONF.
    # A low byte that is not 0xdc stops both motors, re-initialises both drivers and re-homes
    def _gconf_check(self, eventtime):
        self.afc.reactor.register_callback(self._gconf_verify)
        printing = self.printer.lookup_object("print_stats").get_status(eventtime)["state"] == "printing"
        return eventtime + (1. if printing else 0.2)

    def _gconf_verify(self, eventtime):
        if self.printer.is_shutdown() or self.auto_load_busy:
            return
        for name in ("IFS_Drive", "IFS_Selector"):
            driver = self.printer.lookup_object("tmc2209 AFC_stepper " + name, None)
            if driver is None:
                return
            try:
                value = driver.mcu_tmc.get_register("GCONF")
            except self.printer.command_error:
                return
            if (value & 0xff) != 0xdc:
                self._recover_drivers()
                return

    def _recover_drivers(self):
        self.logger.info("IFS driver reset detected, re-initialising both drivers and re-homing")
        self._release(self.drive_stepper_obj)
        self._release(self.selector_stepper_obj)
        self._gcode("INIT_TMC STEPPER=IFS_Drive", "INIT_TMC STEPPER=IFS_Selector")
        self.selector_pos = None
        self.return_to_home(prep=True)

    # stock SysTick: every 20 ms the ring takes the last stored average
    def _odometer_sample(self, eventtime):
        for odo, counts in zip(self.odo, self.speed_latch):
            odo.feed(counts)
        return eventtime + SPEED_TICK

    def _feed_started(self, lane):
        if 0 < lane.index <= len(self.odo):
            self.odo[lane.index - 1].count = 0

    # stock slotIFSMottorNoRunning: while printing, E advances but the roller of the loaded lane does
    # not move for more than 200 polls (1 s each) pauses the print with E0109
    def _odometer_tick(self, eventtime):
        if not self.odometer_enable or not self.odo:
            return eventtime + 1.
        state = self.printer.lookup_object("print_stats").get_status(eventtime)["state"]
        lane = self.afc.function.get_current_lane_obj()
        if (state != "printing" or self.afc.in_toolchange
                or lane is None or lane.unit_obj is not self or lane.index > len(self.odo)):
            return eventtime + 1.
        e = self.printer.lookup_object("gcode_move").get_status(eventtime)["gcode_position"][3]
        if lane.raw_load_state and e > 0 and e != self.odo_last_e:
            self.odo_last_e = e
            i = lane.index - 1
            self.odo_count[i] = 0 if self.odo[i].flag else self.odo_count[i] + 1
            if self.odo_count[i] > 200:
                self.odo_count[i] = 0
                self.afc.reactor.register_callback(lambda ev, n=lane.index: self.afc.error.AFC_error(
                    "E0109: IFS odometer roller {} not moving".format(n), True))
        return eventtime + 1.

    # stock FFS_state 126: PB5 low and PB6 high until the next successful home (F15)
    def _leds(self, error):
        self.ifs_gcode.run_script_from_command("SET_PIN PIN=ifs_pb5 VALUE=%d\nSET_PIN PIN=ifs_pb6 VALUE=%d" % (
            0 if error else 1, 1 if error else 0))

    # stock drops EN the moment a move ends or stops; Kalico's own disable waits 100 ms first
    def _release(self, stepper):
        line = stepper.stepper_enable.enable_lines.get("AFC_stepper " + stepper.name)
        if line is None or not line.is_motor_enabled():
            return
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.flush_step_generation()
        pin = line.enable.mcu_enable
        gap = pin.get_mcu().clock_to_print_time(pin._last_clock) + 0.100 - toolhead.get_last_move_time()
        if gap > 0.:
            toolhead.dwell(gap)
        line.motor_disable(toolhead.get_last_move_time())
        toolhead.dwell(0.100)

    def _gcode(self, *lines):
        self.ifs_gcode.run_script_from_command("\n".join(lines))

    @staticmethod
    def _material(lane):
        return str(lane.material or "PLA").upper()

    # stock sends the IFS move and the extruder move together, so the lane follows the extruder
    def _extrude_with_lane(self, lane, extruder, amount, feed):
        stepper = self.drive_stepper_obj
        lane.select_lane()
        stepper.extruder_stepper.sync_to_extruder(extruder.th_extruder_name)
        try:
            self._gcode("SAVE_GCODE_STATE NAME=_adx_extrude", "M82", "G92 E0",
                        "G1 E%.3f F%.0f" % (amount, feed), "M400",
                        "RESTORE_GCODE_STATE NAME=_adx_extrude")
        finally:
            stepper.extruder_stepper.sync_to_extruder(None)
            self._release(stepper)

    # stock flushFilamentSecond: fan, 3 s, one extruder flush of 90 mm, fan after
    def _flush_second(self, material):
        feed = FLUSH_SPEED.get(material, 300.)
        fan_first = FLUSH_FAN.get(material, 0)
        fan_after = AFTER_FAN.get(material, 125)
        self._gcode("SAVE_GCODE_STATE NAME=_adx_second", "M82", "M106 S%d" % fan_first, "G4 P3000",
                    "G92 E0", "G1 E90 F%.0f" % feed, "M400", "M106 S%d" % fan_after,
                    "RESTORE_GCODE_STATE NAME=_adx_second")

    # stock flushFilamentBrush: three Y passes through the brush, E-3 first on the second brush
    def _brush_passes(self, second):
        cmds = ["SAVE_GCODE_STATE NAME=_adx_brush", "M82", "G4 P3000"]
        if second:
            cmds += ["G92 E0", "G1 E-3 F300", "M400", "G92 E0"]
        cmds += ["G1 Y%.3f F12000" % self.chute_y_out, "G1 Y%.3f F3000" % self.chute_y,
                 "G1 Y%.3f F12000" % self.chute_y_out, "G1 Y%.3f F3000" % self.chute_y,
                 "G1 Y%.3f F12000" % self.chute_y_out]
        if not second:
            cmds += ["G1 X%.3f F12000" % self.chute_x, "G1 Y%.3f F3000" % self.chute_y]
        cmds += ["M400", "RESTORE_GCODE_STATE NAME=_adx_brush"]
        self._gcode(*cmds)

    # stock cleanFilamentBox: first flush with the lane pushing, brush, second flush, brush
    def _clean_filament_box(self, lane, material):
        feed = FLUSH_SPEED.get(material, 300.)
        fan_first = FLUSH_FAN.get(material, 0)
        fan_after = AFTER_FAN.get(material, 125)
        self._gcode("SAVE_GCODE_STATE NAME=_adx_flush", "G90",
                    "G1 X%.3f F12000" % self.chute_x, "G1 Y%.3f F3000" % self.chute_y, "M400",
                    "M106 S%d" % fan_first)
        self._extrude_with_lane(lane, lane.extruder_obj, 90., feed)
        self.return_to_home()
        self._gcode("M106 S%d" % fan_after)
        self._brush_passes(False)
        self._flush_second(material)
        self._brush_passes(True)
        self._gcode("RESTORE_GCODE_STATE NAME=_adx_flush")

    # stock doFFMLoad: heat for the type, park at the chute, clear old filament, feed the lane to the
    # toolhead sensor, purge, then fans, motors and nozzle off
    def cmd_ADX_LOAD_HEAD(self, gcmd):
        idx = gcmd.get_int("LANE", minval=1, maxval=len(self.silk_pins))
        lane = self._lane_for_index(idx)
        if lane is None or not lane.load_state:
            raise gcmd.error("lane{} has no filament".format(idx))
        material = self._material(lane)
        temp = gcmd.get_float("TEMP", STOCK_TEMP.get(material, 230.))
        if not self.afc.function.is_homed():
            self._gcode("G28")
        self._gcode("M104 S%.0f" % temp)
        self._chute_park()
        heater = self.printer.lookup_object("extruder").get_heater()
        while True:
            now = self.afc.reactor.monotonic()
            current, target = heater.get_temp(now)
            if target > 0 and target <= current <= target + 19.:
                break
            if self.printer.is_shutdown():
                return
            self.afc.reactor.pause(now + 1.)
        loaded = lane.extruder_obj.lane_loaded
        if loaded is not None and loaded != lane.name:
            if not self.afc.TOOL_UNLOAD(self.afc.lanes[loaded], set_start_time=False):
                raise gcmd.error("failed to unload {} before loading {}".format(loaded, lane.name))
        if lane.get_toolhead_pre_sensor_state():
            if loaded is None:
                raise gcmd.error("filament is in the head but no lane is marked loaded: IFS_SET_LOADED LANE=<the lane it came from>, then retry")
            for _ in range(2):
                self.afc.do_tool_cut_tip_form(lane, lane.extruder_obj)
                if self._retract_until_clear(lane, 1200.):
                    break
            else:
                raise gcmd.error("E0113: filament detected in the extruder, check feeding position, clean manually or check the sensor")
        try:
            homed, dist, warn = lane.move_to(1000., SpeedMode.LONG, endstop=lane.get_toolhead_endstop())
        finally:
            self._release(self.drive_stepper_obj)
        if not homed:
            raise gcmd.error("E{:04d}: channel {} feeding timeout".format(99 + lane.index, lane.index))
        self._clean_filament_box(lane, material)
        self._gcode("SET_LANE_LOADED LANE={}".format(lane.name))
        self._gcode("M106 S0")
        if not self.afc.function.is_printing():
            self._gcode("M18")
        self._gcode("M104 S0")

    # stock cutFilament: the lane follows one continuous extruder retract of 60. Once a second the
    # toolhead sensor is read and the first empty reading stops the lane (F112) while the extruder
    # carries on; steps already generated, at most about half a second, still run
    def _retract_until_clear(self, lane, speed):
        stepper = self.drive_stepper_obj
        reactor = self.afc.reactor
        lane.select_lane()
        stepper.extruder_stepper.sync_to_extruder(lane.extruder_obj.th_extruder_name)

        def poll(eventtime):
            if lane.get_toolhead_pre_sensor_state():
                return eventtime + 1.
            stepper.extruder_stepper.stepper.set_trapq(None)
            return reactor.NEVER

        timer = reactor.register_timer(poll, reactor.monotonic() + 1.)
        try:
            self._gcode("SAVE_GCODE_STATE NAME=_adx_retract", "M82", "G92 E0",
                        "G1 E%.3f F%.0f" % (-self.unload_cut_dist, speed), "M400",
                        "RESTORE_GCODE_STATE NAME=_adx_retract")
        finally:
            reactor.unregister_timer(timer)
            stepper.extruder_stepper.sync_to_extruder(None)
            self._release(stepper)
        return not lane.get_toolhead_pre_sensor_state()

    # stock doLoadAndUnload, unload half: cutFilament, lane and extruder back 60 until the toolhead
    # sensor clears (second cut once if it does not), then the lane back 70 at S1200
    def unit_unload_lane(self, lane, extruder):
        lane.status = AFCLaneState.TOOL_UNLOADING
        lane.disable_buffer()
        speed = STOCK_UNLOAD.get(self._material(lane), 600.)
        for _ in range(2):
            self.afc.do_tool_cut_tip_form(lane, extruder)
            if self._retract_until_clear(lane, speed):
                break
        else:
            self.afc.error.handle_lane_failure(lane, "E{:04d}: channel {} retracting timeout".format(103 + lane.index, lane.index))
            return False
        try:
            lane.move(-self.unload_ifs_dist, lane.long_moves_speed, lane.long_moves_accel)
        finally:
            self._release(self.drive_stepper_obj)
        lane.set_tool_unloaded(normal_toolchange=True)
        lane.loaded_to_hub = True
        lane.unit_obj.lane_tool_unloaded(lane)
        lane.status = AFCLaneState.NONE
        self.return_to_home()
        self.afc.save_vars()
        return True

    # stock fourInOneCut: front strip first, then along X to the cutter start
    def cmd_ADX_CUT(self, gcmd):
        cut_vars = self.printer.lookup_object("gcode_macro _AFC_CUT_TIP_VARS").variables
        pin_x, pin_y = [float(v) for v in cut_vars["pin_loc_xy"]]
        park_x = pin_x + float(cut_vars["pin_park_dist"])
        end_x = pin_x - float(cut_vars["cut_move_dist"])
        self.ifs_gcode.run_script_from_command("\n".join([
            "SAVE_GCODE_STATE NAME=_ad5x_cut", "G90", "M82", "M400",
            "G1 Y%.3f F18000" % self.cut_approach_y,
            "G1 Y%.3f F1800" % pin_y,
            "G1 X%.3f F12000" % park_x,
            "G1 X%.3f F600" % end_x,
            "M400", "G92 E0", "G1 E-5 F600", "M400",
            "RESTORE_GCODE_STATE NAME=_ad5x_cut"]))

    # prep: stock homes once at power-on and otherwise trusts the silk sensors,
    # so skip AFC_HTLF's re-home per lane and never move lanes during prep
    def system_Test(self, cur_lane, delay, assignTcmd, enable_movement):
        cur_lane.prep_state = cur_lane.load_state
        if not self.prep_homed:
            if not self.return_to_home():
                return False
        status = super(AFC_HTLF, self).system_Test(cur_lane, delay, assignTcmd, False)
        return self.prep_homed and status

    # stock enters and leaves the chute along Y only
    def _chute_park(self):
        pos = self.printer.lookup_object("toolhead").get_position()
        cmds = ["SAVE_GCODE_STATE NAME=_ifs_chute", "G90"]
        if pos[0] != self.chute_x or pos[1] != self.chute_y:
            if pos[1] > self.chute_y_out:
                cmds.append("G1 Y%.3f F3000" % self.chute_y_out)
            cmds += ["G1 X%.3f F12000" % self.chute_x,
                     "G1 Y%.3f F12000" % self.chute_y_out,
                     "G1 Y%.3f F3000" % self.chute_y]
        cmds += ["M400", "RESTORE_GCODE_STATE NAME=_ifs_chute"]
        self.ifs_gcode.run_script_from_command("\n".join(cmds))

    # stock leaves the chute with Y first, then X, then Z; AFC would cut diagonally across the back
    def _restore_pos(self, move_z_first=True):
        pos = self.printer.lookup_object("toolhead").get_position()
        if self.afc.position_saved and pos[1] > self.chute_y_out:
            self.ifs_gcode.run_script_from_command("G90\nG1 Y%.3f F12000\nM400" % self.afc.last_gcode_position[1])
        self.afc_restore_pos(move_z_first)

    # stock flushes at the start of a print even when the first lane is already in the head
    def _change_tool(self, cur_lane, *args, **kwargs):
        afc = self.afc
        if (afc.function.in_print() and afc.current_toolchange == -1 and not afc.error_state
                and cur_lane.unit_obj is self and afc.current == cur_lane.name):
            afc.save_pos()
            self._clean_filament_box(cur_lane, self._material(cur_lane))
            afc.restore_pos()
        result = self.afc_change_tool(cur_lane, *args, **kwargs)
        if afc.function.in_print() and cur_lane.unit_obj is self and not afc.error_state:
            self.return_to_home()
        return result

    # the lane being loaded is not marked loaded yet when AFC purges, so ADX_POOP gets it from here
    def _poop_kick_wipe(self, cur_lane, cur_extruder, purge_length=None):
        self.poop_lane = cur_lane if cur_lane.unit_obj is self else None
        self.poop_extruder = cur_extruder
        try:
            return self.afc_poop_kick_wipe(cur_lane=cur_lane, cur_extruder=cur_extruder, purge_length=purge_length)
        finally:
            self.poop_lane = None

    def cmd_ADX_PARK(self, gcmd):
        self._chute_park()

    def cmd_ADX_POOP(self, gcmd):
        self._chute_park()
        lane = self.poop_lane
        if lane is not None:
            material = self._material(lane)
            self._gcode("M106 S%d" % FLUSH_FAN.get(material, 0))
            self._extrude_with_lane(lane, self.poop_extruder, max(gcmd.get_float("PURGE_LENGTH", 90.), 60.),
                                    FLUSH_SPEED.get(material, 300.))
            self._gcode("M106 S%d" % AFTER_FAN.get(material, 125))
            return
        params =" ".join("%s=%s" % (k, v) for k, v in gcmd.get_command_parameters().items())
        self.ifs_gcode.run_script_from_command("AFC_POOP " + params)

    def cmd_ADX_BRUSH(self, gcmd):
        cmds = ["SAVE_GCODE_STATE NAME=_ifs_brush", "G90",
                "G1 Y%.3f F3000" % self.chute_y_out,
                "G1 X%.3f F12000" % self.brush_x_out,
                "G1 Y%.3f F3000" % self.chute_y]
        for i in range(self.brush_count):
            cmds += ["G1 X%.3f" % self.brush_x_min, "G1 X%.3f" % self.brush_x_max]
        cmds += ["G1 X%.3f" % self.brush_x_min,
                 "G1 X%.3f" % self.brush_x_out,
                 "G1 Y%.3f" % self.chute_y_out,
                 "M400", "RESTORE_GCODE_STATE NAME=_ifs_brush"]
        self.ifs_gcode.run_script_from_command("\n".join(cmds))

    def get_status(self, eventtime=None):
        response = super().get_status(eventtime)
        response["silk"] = list(self.silk_raw)
        response["silk_limits"] = [dict(lim) for lim in self.silk_limits]
        response["speed"] = list(self.speed_latch)
        return response

    def _silk_callback(self, idx, read_time, value):
        self.silk_raw[idx] = int(value * 4095.)
        present = self.silk_state[idx]
        limits = self.silk_limits[idx]
        counts = value * 4095.
        if counts < limits["present_low"] or counts > limits["present_high"]:
            present = True
        elif limits["empty_low"] <= counts <= limits["empty_high"]:
            present = False
        if present is None or present == self.silk_state[idx]:
            return
        lane = self._lane_for_index(idx + 1)
        if lane is None:
            return
        self.silk_state[idx] = present
        eventtime = self.afc.reactor.monotonic()
        self.afc.reactor.register_callback(
            lambda e, l=lane, s=present: self._set_lane_load(l, e, s))

    def _options_load(self):
        try:
            with open(self.options_file) as f:
                self.end_unload = bool(json.load(f).get("end_unload", self.end_unload))
        except (OSError, ValueError, AttributeError):
            pass

    def _options_save(self):
        try:
            os.makedirs(os.path.dirname(self.options_file), exist_ok=True)
            with open(self.options_file + ".tmp", "w") as f:
                json.dump({"end_unload": self.end_unload}, f)
            os.replace(self.options_file + ".tmp", self.options_file)
        except OSError as e:
            self.logger.error("IFS options not saved: {}".format(e))

    def cmd_ADX_END_UNLOAD_SET(self, gcmd):
        value = gcmd.get_int("ENABLE", None, minval=0, maxval=1)
        if value is not None:
            self.end_unload = bool(value)
            self._options_save()
        gcmd.respond_info("end of print unload is {}".format("on" if self.end_unload else "off"))

    # stock end of print: cut and unload the lane in the toolhead; never lets a failure stop the end macro
    def cmd_ADX_END_UNLOAD(self, gcmd):
        if not self.end_unload:
            return
        name = self.afc.current
        lane = self.afc.lanes.get(name) if name else None
        if lane is None or lane.unit_obj is not self:
            return
        try:
            if not self.afc.TOOL_UNLOAD(lane, set_start_time=False):
                gcmd.respond_info("end of print unload of {} failed".format(lane.name))
        except Exception as e:
            self.logger.error("end of print unload of {} failed: {}".format(lane.name, e))
        self.return_to_home()

    def _silk_load(self):
        try:
            with open(self.silk_file) as f:
                data = json.load(f)
        except (OSError, ValueError):
            return
        for key, limits in data.items():
            idx = int(key) - 1
            if 0 <= idx < len(self.silk_limits):
                self.silk_limits[idx].update({k: float(v) for k, v in limits.items() if k in self.silk_default})

    def _silk_save(self):
        data = {str(i + 1): lim for i, lim in enumerate(self.silk_limits) if lim != self.silk_default}
        try:
            os.makedirs(os.path.dirname(self.silk_file), exist_ok=True)
            with open(self.silk_file + ".tmp", "w") as f:
                json.dump(data, f)
            os.replace(self.silk_file + ".tmp", self.silk_file)
        except OSError as e:
            self.logger.error("IFS silk limits not saved: {}".format(e))

    def _silk_lane(self, gcmd):
        lane = gcmd.get_int("LANE", minval=1, maxval=len(self.silk_pins))
        return lane - 1

    def cmd_ADX_SILK_STATUS(self, gcmd):
        for i, lim in enumerate(self.silk_limits):
            gcmd.respond_info("lane{}: raw {} present<{:.0f} empty {:.0f}-{:.0f} present>{:.0f}{}".format(
                i + 1, self.silk_raw[i], lim["present_low"], lim["empty_low"], lim["empty_high"],
                lim["present_high"], "" if lim == self.silk_default else " (custom)"))

    # two point calibration: the limits go halfway between the empty and the present reading
    def cmd_ADX_SILK_CALIBRATE(self, gcmd):
        idx = self._silk_lane(gcmd)
        state = gcmd.get("STATE").lower()
        if state not in ("empty", "present"):
            raise gcmd.error("STATE must be empty or present")
        samples = []
        for _ in range(20):
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.1)
            samples.append(self.silk_raw[idx])
        self.silk_cal[idx][state] = sum(samples) / len(samples)
        gcmd.respond_info("lane{} {} reads {:.0f} (min {}, max {})".format(
            idx + 1, state, self.silk_cal[idx][state], min(samples), max(samples)))
        cal = self.silk_cal[idx]
        if "empty" not in cal or "present" not in cal:
            gcmd.respond_info("now calibrate the other state for lane{}".format(idx + 1))
            return
        empty, present = cal["empty"], cal["present"]
        if abs(empty - present) < 100:
            raise gcmd.error("empty and present readings are too close to separate ({:.0f} and {:.0f})".format(empty, present))
        mid = (empty + present) / 2.
        margin = max(25., abs(empty - present) * 0.05)
        limits = {"present_low": -1., "empty_low": 0., "empty_high": 4095., "present_high": 4096.}
        if present < empty:
            limits["present_low"] = mid - margin
            limits["empty_low"] = mid + margin
        else:
            limits["empty_high"] = mid - margin
            limits["present_high"] = mid + margin
        self.silk_limits[idx] = limits
        self.silk_cal[idx] = {}
        self._silk_save()
        gcmd.respond_info("lane{} limits set: {}".format(idx + 1, limits))

    def cmd_ADX_SILK_SET(self, gcmd):
        idx = self._silk_lane(gcmd)
        limits = dict(self.silk_limits[idx])
        for key in limits:
            value = gcmd.get_float(key.upper(), None, minval=-1., maxval=4096.)
            if value is not None:
                limits[key] = value
        self.silk_limits[idx] = limits
        self._silk_save()
        gcmd.respond_info("lane{} limits set: {}".format(idx + 1, limits))

    def cmd_ADX_SILK_RESET(self, gcmd):
        lane = gcmd.get("LANE", "all")
        for i in range(len(self.silk_limits)):
            if lane == "all" or int(lane) == i + 1:
                self.silk_limits[i] = dict(self.silk_default)
        self._silk_save()
        gcmd.respond_info("silk limits reset to stock for {}".format("all lanes" if lane == "all" else "lane" + lane))

    def _set_lane_load(self, lane, eventtime, state):
        lane.load_callback(eventtime, state)
        lane.handle_load_runout(eventtime, state)

    def on_filament_insert(self, lane):
        super().on_filament_insert(lane)
        if self.auto_load:
            self.afc.reactor.register_callback(lambda e, l=lane: self._auto_load(l))

    # stock fourInOneLoadFristStep and SecondStep: one feed of up to 650 mm to the toolhead sensor,
    # then the lane back 100 mm, then release
    def _auto_load(self, lane):
        if self.auto_load_busy or self.afc.function.is_printing(check_movement=True):
            return
        occupied = lane.get_toolhead_pre_sensor_state()
        self.auto_load_busy = True
        self._feed_started(lane)
        try:
            if occupied:
                lane.select_lane()
                lane.move(self.auto_load_max, lane.long_moves_speed, lane.long_moves_accel)
                result = "head occupied, fed {:.0f}mm".format(self.auto_load_max)
            else:
                homed, dist, warn = lane.move_to(self.auto_load_max, SpeedMode.LONG, endstop=lane.get_toolhead_endstop())
                result = "toolhead sensor after {:.0f}mm".format(dist) if homed else "did not reach the toolhead sensor"
            lane.move(-self.park_dist, lane.long_moves_speed, lane.long_moves_accel)
            self.logger.info("{}: auto load {}, parked {:.0f}mm back".format(lane.name, result, self.park_dist))
        finally:
            self._release(self.drive_stepper_obj)
            self.return_to_home()
            self.auto_load_busy = False

    def _lane_for_index(self, index):
        for lane in self.lanes.values():
            if lane.index == index:
                return lane
        return None

    def _home(self):
        if self.home_state:
            self.selector_stepper_obj.move(self.home_backoff * MoveDirection.NEG,
                                           self.selector_movement_speed,
                                           self.selector_movement_accel,
                                           False)
            if self.home_state:
                self.failed_to_home = True
                self._leds(True)
                self.afc.error.AFC_error("E0114: IFS home error, failed to leave home sensor on {}".format(self.name), False)
                return False

        homed, self._homed_distance = self.selector_stepper_obj.do_homing_move(
            self.home_search * MoveDirection.POS,
            self.selector_movement_speed,
            self.selector_movement_accel,
            self.home_endstop_name,
            assist_active=False
        )
        if not homed:
            self.failed_to_home = True
            self._leds(True)
            self.afc.error.AFC_error("E0114: IFS home error, failed to home {}".format(self.name), False)
            return False

        self._leds(False)
        self.failed_to_home = False
        self.prep_homed = True
        self.selector_pos = 0.
        return True

    # stock homes once and then moves by position; slot 1 is at the home sensor
    def _ensure_homed(self, prep=False):
        if prep or self.failed_to_home or self.selector_pos is None:
            return self._home()
        return True

    def _move_to(self, target):
        dist = target - self.selector_pos
        if abs(dist) > 0.001:
            self.selector_stepper_obj.move(dist,
                                           self.selector_movement_speed,
                                           self.selector_movement_accel,
                                           False)
        self.selector_pos = target

    def return_to_home(self, prep=False, disable_selector=True):
        if not self._ensure_homed(prep):
            return False
        if disable_selector:
            # park between slots so no lane is held
            neutral = self.neutral_angle
            if self.last_index is not None:
                neutral += (self.last_index - 1) * self.slot_angle
            self._move_to(neutral)
            self._release(self.selector_stepper_obj)
        else:
            self._move_to(0.)
        self.current_selected_lane = None
        return True

    def calculate_lobe_movement(self, lane_index:int):
        return (lane_index - 1) * self.slot_angle

    def select_lane(self, lane, disable_selector=True):
        if "stepper" in lane.fullname.lower():
            return False, 0.0
        try:
            if self.current_selected_lane != lane:
                if not self._ensure_homed(self.rehome_on_select):
                    self.logger.error("IFS: failed to home when selecting {}".format(lane.name))
                    return False, 0.0
                self._move_to(self.calculate_lobe_movement(lane.index))
                self.last_index = lane.index
                self.current_selected_lane = lane
                return True, self._homed_distance
        finally:
            if disable_selector:
                self._release(self.selector_stepper_obj)
        return True, 0.0

def load_config_prefix(config):
    return AFC_IFS(config)
