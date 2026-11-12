# FlashForge AD5X analog filament sensor (toolhead), for AFC u1_filament_sensor_name
#
# This file may be distributed under the terms of the GNU GPLv3 license.

SAMPLE_TIME  = 0.001
SAMPLE_COUNT = 8
REPORT_TIME  = 0.100
ENDSTOP_POLL_TIME = 0.010

class RunoutHelper:
    def __init__(self, config):
        self.name = config.get_name().split()[-1]
        self.reactor = config.get_printer().get_reactor()
        self.event_delay = config.getfloat("event_delay", 3.0, minval=0.)
        self.min_event_systime = self.reactor.NEVER
        self.filament_present = False
        self.sensor_enabled = True
        self.runout_pause = False
        self.runout_gcode = None
        self._runout_event_handler = None

    def note_filament_present(self, state, force=False):
        self.filament_present = state

# Software endstop for homing moves: triggers while the sensor reads filament
class AD5XEndstop:
    def __init__(self, sensor):
        self._sensor = sensor
        self._reactor = sensor.printer.get_reactor()
        self._steppers = []
        self._trigger_time = 0.
        self._completion = None
        self._poll_timer = None

    def get_mcu(self):
        return self._sensor.mcu_adc.get_mcu()

    def add_stepper(self, stepper):
        if stepper not in self._steppers:
            self._steppers.append(stepper)

    def get_steppers(self):
        return list(self._steppers)

    def home_start(self, print_time, sample_time, sample_count, rest_time, triggered=True):
        self._triggered = triggered
        self._trigger_time = 0.
        self._completion = self._reactor.completion()
        if self._sensor.state is not None and bool(self._sensor.state) == triggered:
            self._trigger_time = self.get_mcu().estimated_print_time(self._reactor.monotonic())
            self._completion.complete(True)
        else:
            self._poll_timer = self._reactor.register_timer(self._poll, self._reactor.NOW)
        return self._completion

    def _poll(self, eventtime):
        if self._sensor.state is not None and bool(self._sensor.state) == self._triggered:
            self._trigger_time = self.get_mcu().estimated_print_time(eventtime)
            self._completion.complete(True)
            return self._reactor.NEVER
        return eventtime + ENDSTOP_POLL_TIME

    def home_wait(self, home_end_time):
        if self._poll_timer is not None:
            self._reactor.unregister_timer(self._poll_timer)
            self._poll_timer = None
        return self._trigger_time

    def query_endstop(self, print_time):
        return 1 if self._sensor.state else 0

class AD5XFilamentSensor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.present_below = config.getfloat("present_below", 0.3)
        self.present_above = config.getfloat("present_above", 0.72)
        self.runout_helper = RunoutHelper(config)
        self.value = 0.
        self.state = None
        self.endstop = AD5XEndstop(self)
        pin = config.get("sensor_pin")
        ppins = self.printer.lookup_object("pins")
        ppins.allow_multi_use_pin(pin)
        self.mcu_adc = ppins.setup_pin("adc", pin)
        self.mcu_adc.setup_adc_sample(SAMPLE_TIME, SAMPLE_COUNT)
        self.mcu_adc.setup_adc_callback(REPORT_TIME, self._adc_callback)
        self.printer.add_object("filament_motion_sensor %s" % (self.name,), self)

    def _adc_callback(self, read_time, value):
        self.value = value
        state = value <= self.present_below or value >= self.present_above
        if state == self.state:
            return
        self.state = state
        self.runout_helper.note_filament_present(state)

    def get_status(self, eventtime):
        return {"filament_detected": bool(self.runout_helper.filament_present),
                "enabled": bool(self.runout_helper.sensor_enabled),
                "value": self.value}

def load_config_prefix(config):
    return AD5XFilamentSensor(config)
