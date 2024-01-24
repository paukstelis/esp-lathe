#this requires micropython version that has machine.Counter to use hardware counting for tachometer
#other libraries used:
#umenu
#uota
#st7789
#rotary_irq currently, but likely will be changed to use machine.Encoder
#wificonfigurator mostly via ChatGTP
#Peter Hinch Pushbutton primitive

from machine import Pin, SPI
import time, math
from machine import SoftI2C, reset
import uasyncio as asyncio
import network
import espnow
import json
import binascii

from vfd import VFD
from tachometer import tachometer
from mpu6050 import MPU6050
from primitives.pushbutton import Pushbutton
from rotary_irq_esp import RotaryIRQ

import st7789
import vga1_16x16 as font
import vga2_8x8 as smallfont

#Load config
config = []
with open("settings.json") as f:
    config = json.load(f)
for s in config["settings"]:
    if s["id"] == "use_angle":
        USEACCEL = s["value"]
    if s["id"] == "angle_threshold":
        ANGLE_THRESHOLD = s["value"]
    if s["id"] == "angle_axis":
        AXIS = s["value"]
    if s["id"] == "wired_control":
        WIRES = int(s["value"])
    if s["id"] == "encoder_sensitivity":
        ENCSTEPS = s["value"]
    if s["id"] == "circ_diam":
        WHEEL_DIAM = s["value"]
    if s["id"] == "use_tach":
        TACH = s["value"]
    if s["id"] =="tach_reads":
        TACH_READS = s["value"]
    if s["id"] == "power_down":
        POWERDOWN = s["value"]*60000
    if s["id"] == "remote_timeout":
        SLEEP = s["value"]*60000
    if s["id"] == "remote_ping":
        REM_PING = s["value"]
    if s["id"] == "remote_debug":
        REM_DEBUG = s["value"]
    if s["id"] == "accel_reads":
        ACCEL_READS = s["value"]
    if s["id"] == "rem_ping":
        REM_PING = s["value"]

#Analog output
vfd = VFD(26)
vfd.set_dac(0)

#NPN pins
spindle = Pin(4, Pin.OUT) #alias as forward
direction = Pin(16, Pin.OUT) #alias as reverse
brake = Pin(17, Pin.OUT, Pin.PULL_UP) #This will be jumpered with a pushbutton to activate the 2N7000
brake_active = Pin(5, Pin.IN, Pin.PULL_UP)

#local encoder
r = RotaryIRQ(pin_num_clk=12, pin_num_dt=13, incr=ENCSTEPS, min_val=0, max_val=255, pull_up=True, range_mode=RotaryIRQ.RANGE_BOUNDED)
#TODO, don't use 12. It is bootstrap pin so if pulled up, flash will fail
#accelerometer
#MPU
mpu = None
if USEACCEL:
    try:
        if config["mpu_ofs"]["complete"] == False:
            mpu = MPU6050(0, 21, 22)
            ofs1 = mpu.__readWords(0x6   , 3)
            ofs2 = mpu.__readWords(0x13, 3)
            mpu_ofs = ofs1 + ofs2
            #write ofs to settings file
            config["mpu_ofs"]["ofs"] = mpu_ofs
            config["mpu_ofs"]["complete"] = True
            with open("settings.json","w") as jsonfile:
                json.dump(config, jsonfile)
        else:
            mpu_ofs = config["mpu_ofs"]["ofs"]
            print(mpu_ofs)
            mpu = MPU6050(0, 21, 22, tuple(mpu_ofs), 34)
    except:
        print("MPU error")
        USEACCEL = False
ANGLE = 0
roll = 0
pitch = 0

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

#PWRLAG = Powermatic/Laguna
if WIRES < 3:
    PWRLAG = True
else:
    PWRLAG = False

#TODO - add some message on the screen? Give IP when configured?
if not config["firstrun"]["complete"]:
    v=10
    msg="Network Setup"
    tft.text(smallfont, "{}".format(msg),3,v,WHITE,BLACK)
    v += font.HEIGHT
    msg="Connect to ESP-Lathe"
    tft.text(smallfont, "{}".format(msg),3,v,WHITE,BLACK)
    v += font.HEIGHT
    msg="http://192.168.4.1"
    tft.text(smallfont, "{}".format(msg),3,v,WHITE,BLACK)
    from wificonfigurator import WiFiConfigurator
    wc = WiFiConfigurator()
    wc.start_ap_web_interface()

sta = network.WLAN(network.STA_IF)
sta.active(True)
sta.disconnect()

e = espnow.ESPNow()
e.active(True)
remotes = []
if len(config["remote"]):
    for remote in config["remote"]:
        addr = binascii.unhexlify(remote.replace(':', ''))
        remotes.append(addr)
        e.add_peer(addr)
        e.send(addr, "UHM{}".format(config["mac"]["address"]))

central_control = b"H'\xe2N3(" #This can be moved to an option during network setup
e.add_peer(central_control) #Central control will have MQTT, etc. for HA integration

RUNNING = False
BRAKE = False
ESTOP = False
DIRECTION = 0
VFDVAL = 0
LAST_STOP = time.ticks_ms()
REM_DEBUG = False
REM_CONNECTED = False

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

def vfd_stop():
    global RUNNING, LAST_STOP
    if not MODE:
        if PWRLAG:
            direction.off()
        spindle.off()
        RUNNING = False
        LAST_STOP = time.ticks_ms()
        e.send(central_control, "L0", False)

    gc.collect()

#unused
def vfd_e_stop():
    global ESTOP
    spindle.off()
    ESTOP = True

def vfd_start():
    global RUNNING
    if ESTOP:
        return
    vfd.set_dac(VFDVAL)
    if not MODE:
        if PWRLAG:
            if not DIRECTION:
                spindle.off()
                direction.on()
            if DIRECTION:
                direction.off()
                spindle.on()
        else:
            direction.value(DIRECTION)
            spindle.on()
        
        RUNNING = True
        print("Running: {}".format(RUNNING))
        e.send(central_control, "L1", False)

def remote_connected():
    global REM_CONNECTED
    print("Remote control sent connect message")
    if MODE:
        reset()
    #send info to remote
    #hostmac = "UHM{}".format(config["mac"]["address"])
    timeout = "URT{}".format(SLEEP)
    debug = "URD{}".format(REM_DEBUG)
    encsteps = "UES{}".format(ENCSTEPS)
    for remote in remotes:
        e.send(remote,hostmac, False)        
        e.send(remote,timeout, False)
        e.send(remote,debug, False)
        e.send(remote,encsteps, False)
        e.send(remote, "{}".format(VFDVAL), False)
    e.send(central_control,"Remote Connected", False)
    REM_CONNECTED = True

async def update_RPM():
    global RPM
    while True and TACH:
        if MODE == 2:
            break
        RPM = tach.get_RPM()
        await asyncio.sleep_ms(50)

async def update_display():
    global RUNNING, BRAKE, RPM, AcBASE, AcMax, DIRECTION
    v = 10
    #normal mode list
    headings = ["Out:  ", "Run:  "]
    if TACH:
        headings.extend(["RPM:   "])
    if mpu:
        headings.extend(["T:  ","Ang:  "])

    for heading in headings:
        tft.text(font, "{}".format(heading),3,v,WHITE,BLACK)
        v += font.HEIGHT

    while True:
        v = 10
        if not MODE:
            percent = int((VFDVAL*100)/255)

            thedir = None

            if RUNNING:
                if DIRECTION:
                    thedir = "For."
                else:
                    thedir = "Rev."
            elif BRAKE:
                thedir = "Brk."
            else:
                thedir = "Stop"

            if abs(ANGLE - ANGLE_THRESHOLD) < 5:
                ACCELCOLOR = st7789.RED
            else:
                ACCELCOLOR = WHITE
            normal = [{'name': "percent", 'value': "{}%    ".format(percent), 'color': WHITE},
                      {'name': "running", 'value': "{}     ".format(thedir), 'color': WHITE}
                     ]
            if TACH:
                normal.extend([{'name': "rpm", 'value': "{}     ".format(RPM), 'color': WHITE}])
        
            if mpu:
                normal.extend([{'name': "temp", 'value': "{:.1f}     ".format(TEMPERATURE), 'color': WHITE},
                              {'name': "accel", 'value': "{:.2f}     ".format(ANGLE), 'color': ACCELCOLOR}])

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
    global mpu, ANGLE, TEMPERATURE, roll, pitch
    #This may need a lot of work to figure out when to shut it down
    axis = AXIS
    while True:
        if not mpu:
            return
        if MODE == 2:
            break
        roll, pitch = mpu.angles
        TEMPERATURE = mpu.fahrenheit
        ANGLE = eval(axis)
        if abs(ANGLE) > ANGLE_THRESHOLD and RUNNING:
                vfd_stop()
                print("Stopped from acceleration")
                e.send(central_control, "A{}".format(ANGLE), False)
        await asyncio.sleep_ms(10)

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
    global BRAKE, RUNNING
    while True:
        if not RUNNING and not brake_active.value():
            #logic may need reversal here
            brake.on()
            BRAKE = True
            #brake.on()
        else:
            #logic may need reverasl here
            brake.off()
            BRAKE = False
            #brake.off()
        await asyncio.sleep_ms(10)

async def ping_remote():
    while True:
        if REM_CONNECTED and REM_PING:
            for remote in remotes:
                if e.send(remote, "P", sync=True):
                    print("remote alive")
                else:
                    if RUNNING:
                        vfd_stop()
        await asyncio.sleep_ms(REM_PING*1000)

async def get_message():
    global r, vfd
    while True:
        #print("Getting message...")
        if MODE == 2:
            break
        host, msg = e.recv(timeout_ms=20)
        if msg:             # msg == None if timeout in recv()
            #print(host, msg)
            msgd = msg.decode("utf-8")
            if msgd.startswith('F'):
                handle_forward()
            elif msgd.startswith('R'):
                handle_reverse()
            elif msgd.startswith('C'):
                remote_connected()
            elif msgd.startswith('E'):
                toggle_circ()
            elif msgd.startswith('B'):
                print("Got brake message")
            elif msgd.startswith('V'):
                encoder_counts = int(msgd[1:])
                update_circ(encoder_counts)
            elif msgd.isdigit():
                #Only get here if it is int
                VFDVAL = int(msg)
                if VFDVAL > 255:
                    VFDVAL = 255
                vfd.set_dac(VFDVAL)
                #reset local value
                r.set(int(VFDVAL))

        await asyncio.sleep_ms(25)

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
    asyncio.create_task(brake_check())
    #asyncio.create_ask(ping_remote())

    while True:
        if not MODE:
            #after X minutes since the last stop, reset speed to minimize accidents
            if not RUNNING and time.ticks_diff(time.ticks_ms(), LAST_STOP) > POWERDOWN:
                r.set(0)
                LAST_STOP = time.ticks_ms()
            val_new = r.value()
            if val_new != VFDVAL:
                VFDVAL = val_new
                vfd.set_dac(VFDVAL)
                print(VFDVAL)
                for remote in remotes:
                    e.send(remote, "{}".format(VFDVAL), False)
                LAST_STOP = time.ticks_ms()
        await asyncio.sleep_ms(10)

if __name__ == '__main__':
    gc.enable()
    asyncio.run(main())

#GPIO available
#15
#1 tx
#3 rx
#19
#27
#GPIOs used
#2
#4
#5
#12
#13
#16
#17
#18
#21
#22
#23
#25
#26
#32
#33
#34 input only
#35 input only
