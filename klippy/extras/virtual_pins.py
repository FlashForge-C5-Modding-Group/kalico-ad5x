# Virtual Pins support
#
# Copyright (C) 2023 Pedro Lamas <pedrolamas@gmail.com>
#
# This file may be distributed under the terms of the MIT license.
#
# From https://github.com/pedrolamas/klipper-virtual-pins (commit e0013c4) for the OpenAD5X project
# Edited by sterling motuz <Sterling@dexabyte.ca> for the OpenAD5X project: on/off output pins only

class VirtualPins:
    def __init__(self, config):
        self._printer = config.get_printer()
        self._ppins = self._printer.lookup_object('pins')
        self._ppins.register_chip('virtual_pin', self)
        self._pins = {}

    def setup_pin(self, pin_type, pin_params):
        name = pin_params['pin']
        if pin_type != 'digital_out':
            raise self._ppins.error(
                "virtual pin '%s': only on/off output pins are supported" % (
                    name,))
        if name in self._pins:
            return self._pins[name]
        pin = DigitalOutVirtualPin(self._printer, pin_params)
        self._pins[name] = pin
        return pin

    def get_status(self, eventtime):
        return {
            'pins': {
                name : pin.get_status(eventtime)
                    for name, pin in self._pins.items()
            }
        }

class DigitalOutVirtualPin:
    def __init__(self, printer, pin_params):
        self._invert = pin_params['invert']
        self._value = 0.
        self._real_mcu = printer.lookup_object('mcu')

    def get_mcu(self):
        return self._real_mcu

    def setup_max_duration(self, max_duration):
        pass

    def setup_start_value(self, start_value, shutdown_value):
        self._set_value(start_value)

    def set_digital(self, print_time, value):
        self._set_value(value)

    def get_status(self, eventtime):
        return {
            'value': self._value,
            'type': 'digital_out'
        }

    def _set_value(self, value):
        self._value = (not not value) ^ self._invert

def load_config(config):
    return VirtualPins(config)
