from umenu import *
import json
from machine import reset
from primitives.pushbutton import Pushbutton
from rotary_irq_esp import RotaryIRQ
from machine import Pin, Encoder, SPI
import st7789
import vga1_16x16 as font
import uasyncio as asyncio
import os

config = []
with open("settings.json") as f:
    config = json.load(f)


spi = SPI(2,baudrate=40000000, polarity=0, phase=0, sck=Pin(18), mosi=Pin(23), miso=Pin(19))
tft = st7789.ST7789(spi, 128, 160, reset=Pin(33, Pin.OUT), dc=Pin(32, Pin.OUT), color_order=st7789.BGR, rotation=1)
tft.init()
#alias black and white since they are flipped
WHITE=st7789.BLACK
BLACK=st7789.WHITE
tft.fill(BLACK)
menu = Menu(tft, 8, 12)
menu.set_screen(MenuScreen('Settings'))

def network_connect():
    import network, gc, time
    asyncio.new_event_loop()
    gc.collect()
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    #wlan.config(dhcp_hostname="lathe-test")
    wlan.config(dhcp_hostname="lathe")
    endtime = time.ticks_add(time.ticks_ms(), 45000)
    if not wlan.isconnected():
        print('connecting to network...')
        wlan.connect(config["network"]["ssid"], config["network"]["dwssap"])
        while not wlan.isconnected() and time.ticks_diff(endtime, time.ticks_ms()) > 0:
            pass
    print('network config:', wlan.ifconfig())
    return wlan.isconnected()


def copy(s, t):
    try: 
        if os.stat(t)[0] & 0x4000:  # is directory
            t = t.rstrip("/") + "/" + s
    except OSError:
        pass
    with open(s, "rb") as s:
        with open(t, "wb") as t:
            while True:
                l = s.read(512)
                if not l: break
                t.write(l)

def factory_reset():
    copy("settings.factory","settings.json")
    reset()
    
def value_update(id, value):
    global config
    for each in config["settings"]:
        if each["id"] == id:
            each["value"] = value

def toggle_status(id):
    for each in config["settings"]:
        if each["id"] == id:
           return each["value"]

def toggle_update(id):
    for each in config["settings"]:
        if each["id"] == id:
            if each["value"]:
                each["value"] = False
            else:
                each["value"] = True

def save_and_exit():
    with open("settings.json","w") as jsonfile:
        json.dump(config, jsonfile)
    reset()

for each in config["settings"]:
    print(each)
    id = each["id"]
    if each["type"] == "ValueItem":
        menu.main_screen.add(ValueItem(each["name"],each["value"],each["min"],each["max"],each["step"],(value_update, id)))
    if each["type"] == "EnumItem":
        cur_index = each["values"].index(each["value"])
        menu.main_screen.add(EnumItem(each["name"],each["values"], (value_update, id), cur_index))
    if each["type"] == "ToggleItem":
        menu.main_screen.add(ToggleItem(each["name"], (toggle_status,id), (toggle_update,id)))

menu.main_screen.add(CallbackItem("Save and Exit", save_and_exit))
menu.main_screen.add(CallbackItem("Exit", reset))
menu.main_screen.add(CallbackItem("Network", network_connect))
menu.main_screen.add(CallbackItem("Factory Reset", factory_reset))
menu._update_display(menu.main_screen.__dict__['_items'])
menu.draw()

def t1_press():
    #print("clicked")
    menu.click()


async def do_menu():
    global menu
    t1 = Pushbutton(Pin(14, Pin.IN, Pin.PULL_UP), suppress=True)
    t1.release_func(t1_press, ())
    r = RotaryIRQ(pin_num_clk=12, pin_num_dt=13, pull_up=True)
    val_old = r.value()
    while True:
        val_new = r.value()
        if val_old != val_new:
            menu.move(-1 if val_old > val_new else 1)
            val_old = val_new
        await asyncio.sleep_ms(50)

asyncio.run(do_menu())
