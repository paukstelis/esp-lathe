#this requires micropython version that has machine.Counter to use hardware counting for tachometer
#other libraries used:
#umenu
#uota
#st7789
#rotary_irq currently, but likely will be changed to use machine.Encoder
#wificonfigurator


from machine import Pin, SPI
import time, math
from machine import SoftI2C, reset
import uasyncio as asyncio
import network
import espnow
import json

from vfd import VFD
from tachometer import tachometer
from mpu6050 import accel
from primitives.pushbutton import Pushbutton
from rotary_irq_esp import RotaryIRQ

import st7789
import vga1_16x16 as font


#Load config
config = []
with open("settings.json") as f:
    config = json.load(f)
for s in config["settings"]:
    if s["id"] == "use_accel":
        USEACCEL = s["value"]
    if s["id"] == "accel_threshold":
        ACCEL_THRESHOLD = s["value"]
    if s["id"] == "accel_axis":
        AXIS = s["value"]
    if s["id"] == "circ_diam":
        WHEEL_DIAM = s["value"]
    if s["id"] =="tach_reads":
        TACH_READS = s["value"]
    if s["id"] == "power_down":
        POWERDOWN = s["value"]*60000
    if s["id"] == "remote_timeout":
        SLEEP = s["value"]*60000
    if s["id"] == "accel_reads":
        ACCEL_READS = s["value"]

#Analog output
vfd = VFD(26)
vfd.set_dac(0)

#NPN pins
spindle = Pin(4, Pin.OUT)
direction = Pin(16, Pin.OUT)
brake = Pin(17, Pin.OUT, Pin.PULL_UP) #This will be jumpered with a pushbutton to activate the 2N7000
brake_active = Pin(36, Pin.IN, Pin.PULL_UP)

#local encoder
r = RotaryIRQ(pin_num_clk=12, pin_num_dt=13, min_val=0, max_val=51, pull_up=True, range_mode=RotaryIRQ.RANGE_BOUNDED)
#TODO, don't use 12. It is bootstrap pin so if pulled up, flash will fail
#accelerometer
#MPU
mpu_readings = [0] * ACCEL_READS
mpu = None
if USEACCEL:
    try:
        mpu_ready = Pin(34, Pin.IN, Pin.PULL_UP)
        i2c = SoftI2C(scl=Pin(22), sda=Pin(21))
        mpu = accel(i2c)
        Ac_T = ACCEL_THRESHOLD
        TEMPERATURE = mpu.get_values()["Tmp"]
    except:
        print("Acceleometer not found")
        mpu = None

#tachometer
tach = tachometer(35,500,read_value=TACH_READS)
RPM = 0

#screen
spi = SPI(2,baudrate=40000000, polarity=0, phase=0, sck=Pin(18), mosi=Pin(23), miso=Pin(19))
tft = st7789.ST7789(spi, 128, 160, reset=Pin(33, Pin.OUT), dc=Pin(32, Pin.OUT), color_order=st7789.BGR, rotation=1)
tft.init()
#alias black and white since they are flipped
WHITE=st7789.BLACK
BLACK=st7789.WHITE
tft.fill(BLACK)

print(config)
#TODO - add some message on the screen? Give IP when configured?
if not config["firstrun"]["complete"]:
    from wificonfigurator import WiFiConfigurator
    wc = WiFiConfigurator()
    wc.start_ap_web_interface()

sta = network.WLAN(network.STA_IF)
sta.active(True)
sta.disconnect()

e = espnow.ESPNow()
e.active(True)
remote_control = b"H'\xe2M\xf5F"
central_control = b'H\xe7)\xb4\xea\x94' #not yet correct
e.add_peer(remote_control)
e.add_peer(central_control) #Central control will have MQTT, etc. for HA integration

RUNNING = False
ESTOP = False
DIRECTION = 0
VFDVAL = 0
LAST_STOP = time.ticks_ms()

#circuference measurement
MODE = 0 #0 = normal, 1 = circumference measurement, 2 = setting mode
WHEEL_DIAM = WHEEL_DIAM
M_CIRC = 0
M_DIAM = 0
M_RADIUS = 0

def save_config(msgd):
    reset()

def handle_forward():
    global DIRECTION
    if RUNNING:
        vfd_stop()
        return
    DIRECTION = 1
    vfd_start()

def handle_reverse():
    global DIRECTION
    if RUNNING:
        vfd_stop()
        return
    DIRECTION = 0
    vfd_start()

def handle_brake():
    if RUNNING:
        return
    if not MODE:
        start_brake()

def vfd_stop():
    global DIRECTION, RUNNING, LAST_STOP
    if not MODE:
        spindle.off()
        RUNNING = False
        LAST_STOP = time.ticks_ms()

def vfd_e_stop():
    global ESTOP
    spindle.off()
    ESTOP = True

def vfd_start():
    global ESTOP, DIRECTION, VFDVAL, RUNNING, LAST_STOP
    if ESTOP:
        return
    direction.value(DIRECTION)
    vfd.set_dac(VFDVAL*5)
    if not MODE:
        spindle.on()
        RUNNING = True
        print("Running: {}".format(RUNNING))

def remote_connected():
    print("Remote control sent connect message")
    if MODE:
        reset()
    #update remote position
    e.send(remote_control, "{}".format(VFDVAL), False)
    #send timeout value
    timeout = "URT{}".format(SLEEP)
    e.send(remote_control,"{}".format(timeout), False)

def start_brake():
    global RUNNING
    if RUNNING:
        vfd_stop()
    print("Brake started")

async def update_RPM():
    global RPM
    while True:
        if MODE == 2:
            break
        rpm = tach.get_RPM()
        RPM = rpm
        await asyncio.sleep_ms(100)

async def update_display():
    global RUNNING, RPM, AcBASE, AcMax, DIRECTION
    v = 10
    #normal mode list
    headings = ["Out:  ", "Run:  ","RPM:   "]

    if mpu:
        headings.extend(["F:  ","Acc:  "])

    for heading in headings:
        tft.text(font, "{}".format(heading),3,v,WHITE,BLACK)
        v += font.HEIGHT

    while True:
        v = 10
        if not MODE:
            percent = int((VFDVAL*500)/255)

            thedir = None
            if RUNNING:
                if DIRECTION:
                    thedir = "For."
                else:
                    thedir = "Rev."
            else:
                thedir = "Stop"
            if abs(AcMax - ACCEL_THRESHOLD) < 1000:
                ACCELCOLOR = st7789.RED
            else:
                ACCELCOLOR = WHITE
            normal = [{'name': "percent", 'value': "{}%    ".format(percent), 'color': WHITE},
                      {'name': "running", 'value': "{}     ".format(thedir), 'color': WHITE},
                      {'name': "rpm", 'value': "{}     ".format(RPM), 'color': WHITE}
                      ]
            if mpu:
                normal.extend([{'name': "temp", 'value': "{:.1f}     ".format((1.8*TEMPERATURE)+32), 'color': WHITE},
                              {'name': "accel", 'value': "{}     ".format(AcMax), 'color': ACCELCOLOR}])

            for each in normal:
                tft.text(font, each["value"],63,v,each["color"],BLACK)
                v += font.HEIGHT

        elif MODE == 1: #circ mode
            normal = ["{:.2f}     ".format(M_CIRC),
                      "{:.2f}     ".format(M_DIAM),
                      "{:.2f}     ".format(M_RADIUS)]

            for each in normal:
                tft.text(font, each,43,v,WHITE,BLACK)
                v += font.HEIGHT
        elif MODE == 2: #menu mode
            break
        await asyncio.sleep_ms(200)

async def check_accel():
    global mpu, mpu_readings, AcMax, RUNNING, TEMPERATURE
    #This may need a lot of work to figure out when to shut it down

    while True:
        if not mpu:
            return
        if MODE == 2:
            break
        if not mpu_ready.value():
            #print("Checking accel...")
            mpuval = mpu.get_values()
            AcZ = mpuval["Ac{}".format(AXIS)]
            TEMPERATURE = mpuval["Tmp"]
            Ac = abs(AcZ - AcBASE)
            mpu_readings.insert(0,Ac)
            mpu_readings.pop()
            average = sum(mpu_readings)/len(mpu_readings)
            AcMax = int(average)
            if average > ACCEL_THRESHOLD and RUNNING:
                vfd_stop()
                print("Stopped from acceleration")
        await asyncio.sleep_ms(50)

#functions for circuference measurement tool
def update_circ(encoder_counts):
    global M_CIRC, M_RADIUS, M_DIAM
    pi = math.pi
    dist_per_count = (WHEEL_DIAM*pi)/100
    M_CIRC = dist_per_count * encoder_counts
    M_DIAM = M_CIRC/pi
    M_RADIUS = M_DIAM/2

def toggle_circ():
    global MODE
    if MODE:
        reset()
    else:
        MODE = 1
        tft.fill(BLACK)
        v = 10
        tft.text(font, "C:    ",3,v,WHITE,BLACK)
        v += font.HEIGHT
        tft.text(font, "D:    ",3,v,WHITE,BLACK)
        v += font.HEIGHT
        tft.text(font, "R:     ",3,v,WHITE,BLACK)
        update_circ(0)

#Menu
def start_menu():
    gc.collect()
    MODE = 2
    asyncio.new_event_loop()
    import settingmenu

#button functions
def t1_press():
    handle_forward()

def t1_double():
    global RUNNING, DIRECTION
    print("Double")
    if RUNNING:
        vfd_stop()
        return
    if not MODE:
        start_menu()
    if MODE == 2:
        reset()

def t1_long():
    handle_reverse()

def toggle_vac():
    e.send(central_control, "VAC", False)

async def brake_check():
    while True:
        if MODE == 2:
            break
        if not RUNNING and not brake_active.value():
            brake.on()
        else:
            brake.off()
        await asyncio.sleep_ms(50)

async def get_message():
    global r, vfd
    while True:
        #print("Getting message...")
        if MODE == 2:
            break
        host, msg = e.recv(timeout_ms=10)
        if msg:             # msg == None if timeout in recv()
            print(host, msg)
            msgd = msg.decode("utf-8")
            if msgd.startswith('F'):
                #DIRECTION = 1
                handle_forward()
            elif msgd.startswith('R'):
                #DIRECTION = 0
                handle_reverse()
            elif msgd.startswith('C'):
                remote_connected()
            elif msgd.startswith('B'):
                handle_brake()
            elif msgd.startswith('E'):
                toggle_circ()
            elif msgd.startswith('V'):
                encoder_counts = int(msgd[1:])
                update_circ(encoder_counts)

            else:
                #Only get here if it is int
                VFDVAL = int(msg)
                if VFDVAL > 255:
                    VFDVAL = 255
                vfd.set_dac(VFDVAL)
                #reset local value
                r.set(int(VFDVAL/5))

        await asyncio.sleep_ms(50)

time.sleep_ms(1000)
if mpu:
    AcBASE = mpu.get_values()["Ac{}".format(AXIS)]
    AcMax = AcBASE
else:
    AcMax = 0

async def main():
    global VFDVAL, r, LAST_STOP
    t1 = Pushbutton(Pin(14, Pin.IN, Pin.PULL_UP), suppress=True)
    t1.release_func(t1_press, ())
    t1.double_func(t1_double, ())
    t1.long_func(t1_long, ())
    #Other buttons
    vac = Pushbutton(Pin(2, Pin.IN, Pin.PULL_UP), suppress=True)
    vac.release_func(toggle_vac, ())

    #asyncio tasks
    asyncio.create_task(get_message())
    asyncio.create_task(check_accel())
    asyncio.create_task(update_RPM())
    asyncio.create_task(update_display())
    #asyncio.create_task(brake_check())

    while True:
        if not MODE:
            #after X minutes since the last stop, reset speed to minimize accidents
            if not RUNNING and time.ticks_diff(time.ticks_ms(), LAST_STOP) > POWERDOWN:
                r.set(0)
                LAST_STOP = time.ticks_ms()
            val_new = r.value()
            if val_new != VFDVAL:
                VFDVAL = val_new
                vfd.set_dac(VFDVAL*5)
                print(VFDVAL*5)
                e.send(remote_control, "{}".format(VFDVAL), False)
                LAST_STOP = time.ticks_ms()
        await asyncio.sleep_ms(50)

if __name__ == '__main__':
    gc.enable()
    asyncio.run(main())

#GPIO available
#15
#1 tx
#3 rx
#5

#GPIOs used
#2
#4
#12
#13
#16
#17
#18
#19
#21
#22
#23
#25
#26
#27
#32
#33
#34 input only
#35 input only
