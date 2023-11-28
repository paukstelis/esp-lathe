from machine import Pin, Counter
import time


class tachometer:
    def __init__(self, pin1, filter, read_value=5, timeout_value=8000):
        self.counter = Counter(0, Pin(pin1, Pin.IN, Pin.PULL_UP), filter_ns=filter)
        self.counter.irq(handler=self.RPM_handler, trigger=self.counter.IRQ_MATCH1, value=read_value)
        self.last_count = {"count" : 0, "time" : time.ticks_ms()}
        self.base_read = read_value
        self.read_value = read_value
        self.timeout_value = timeout_value
        self.RPM = 0

    def RPM_handler(self,counter):
        self.counter.pause()
        tv = time.ticks_ms()
        delta_time = time.ticks_diff(tv, self.last_count["time"])
        counts_per_time = float((self.read_value)/delta_time)
        self.RPM = int(60000 * counts_per_time)
        self.counter.value(0)
        self.last_count["time"] = time.ticks_ms()
        self.counter.resume()

    def get_RPM(self):
        tv = time.ticks_ms()
        if time.ticks_diff(tv, self.last_count["time"]) > self.timeout_value:
            self.last_count["time"] = time.ticks_ms()
            self.counter.value(0)
            self.RPM = 0

        return self.RPM
