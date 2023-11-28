import time
from machine import Pin
from machine import DAC

class VFD:
    def __init__(self, dac_pin):
        self.dac = DAC(Pin(dac_pin))

    def set_dac(self, value):
        self.dac.write(value)


