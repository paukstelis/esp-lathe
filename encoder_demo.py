from umenu import *
import json
from machine import reset
from primitives.pushbutton import Pushbutton
from rotary_irq_esp import RotaryIRQ
from machine import Pin, Encoder, SPI
import st7789
import vga1_16x16 as font
import uasyncio as asyncio

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
"""
menu.set_screen(MenuScreen('Main Menu')
                .add(ValueItem('Speed', 10, 1, 100, 1, print))
                .add(EnumItem("Mode", ['Option 1', 'Option 2'], print, 0))
)
"""
menu.set_screen(MenuScreen('Settings'))

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
