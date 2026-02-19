# MicroPython tool launcher for Waveshare ESP32-S3-Touch-LCD-3.5-C
# Copy this file to the device as /main.py

import os
import time
import gc

try:
    import sdcard  # optional SPI SD driver (only used if USE_SDMMC=False)
except ImportError:
    sdcard = None

from machine import Pin, I2C, SPI, SDCard, reset

import st7796

# ---- Board pin map (Waveshare ESP32-S3-Touch-LCD-3.5-C) ----
# LCD SPI (ST7796)
LCD_SCK = 5
LCD_MOSI = 1
LCD_MISO = 2  # not used by LCD but keep SPI happy
LCD_DC = 3
LCD_CS = None  # CS is not wired on this board
LCD_W = 320
LCD_H = 480

# I2C bus (TCA9554 + touch)
I2C_SDA = 8
I2C_SCL = 7

# TCA9554
TCA_ADDR = 0x20
TCA_PIN_BL = 0  # backlight
TCA_PIN_RST = 1 # LCD reset

# SD/MMC pins (per Waveshare ESP-IDF demo)
SD_CLK = 11
SD_CMD = 10
SD_D0  = 9

# Optional SPI SD (only if your board is wired for SPI SD)
SD_SCK  = 11
SD_MOSI = 10
SD_MISO = 9
SD_CS   = 33  # change if your board uses a different CS

# Recommended on this board: SD/MMC (1-bit) on GPIO11/10/9.
USE_SDMMC = True

# ---- Minimal TCA9554 driver ----
class TCA9554:
    REG_IN = 0x00
    REG_OUT = 0x01
    REG_POL = 0x02
    REG_CFG = 0x03

    def __init__(self, i2c, addr=0x20):
        self.i2c = i2c
        self.addr = addr

    def _wr(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([val & 0xFF]))

    def _rd(self, reg):
        return self.i2c.readfrom_mem(self.addr, reg, 1)[0]

    def pin_mode(self, pin, is_output):
        cfg = self._rd(self.REG_CFG)
        if is_output:
            cfg &= ~(1 << pin)
        else:
            cfg |= (1 << pin)
        self._wr(self.REG_CFG, cfg)

    def write_pin(self, pin, val):
        out = self._rd(self.REG_OUT)
        if val:
            out |= (1 << pin)
        else:
            out &= ~(1 << pin)
        self._wr(self.REG_OUT, out)

# ---- FT6336 touch driver (I2C 0x38) ----
class FT6336:
    def __init__(self, i2c, addr=0x38):
        self.i2c = i2c
        self.addr = addr

    def get_point(self):
        try:
            n = self.i2c.readfrom_mem(self.addr, 0x02, 1)[0] & 0x0F
            if n == 0:
                return None
            data = self.i2c.readfrom_mem(self.addr, 0x03, 4)
            x = ((data[0] & 0x0F) << 8) | data[1]
            y = ((data[2] & 0x0F) << 8) | data[3]
            return (x, y)
        except OSError:
            return None

# ---- GT911 touch driver (I2C 0x5D or 0x14) ----
class GT911:
    REG_STATUS = 0x814E
    REG_FIRST = 0x8150

    def __init__(self, i2c, addr=0x5D):
        self.i2c = i2c
        self.addr = addr

    def _wr(self, reg, buf):
        self.i2c.writeto_mem(self.addr, reg, buf)

    def _rd(self, reg, n):
        return self.i2c.readfrom_mem(self.addr, reg, n)

    def get_point(self):
        try:
            status = self._rd(self.REG_STATUS, 1)[0]
            if (status & 0x80) == 0:
                return None
            n = status & 0x0F
            if n == 0:
                self._wr(self.REG_STATUS, b"\x00")
                return None
            data = self._rd(self.REG_FIRST, 8)
            x = data[0] | (data[1] << 8)
            y = data[2] | (data[3] << 8)
            self._wr(self.REG_STATUS, b"\x00")
            return (x, y)
        except OSError:
            return None

# ---- Touch mapping (adjust if mirrored) ----
TOUCH_SWAP_XY = False
TOUCH_INVERT_X = False
TOUCH_INVERT_Y = False

def map_touch(p):
    if p is None:
        return None
    x, y = p
    if TOUCH_SWAP_XY:
        x, y = y, x
    if TOUCH_INVERT_X:
        x = LCD_W - 1 - x
    if TOUCH_INVERT_Y:
        y = LCD_H - 1 - y
    # clamp
    if x < 0:
        x = 0
    if y < 0:
        y = 0
    if x >= LCD_W:
        x = LCD_W - 1
    if y >= LCD_H:
        y = LCD_H - 1
    return (x, y)

# ---- Hardware init ----
spi = SPI(2, baudrate=40000000, polarity=0, phase=0,
          sck=Pin(LCD_SCK), mosi=Pin(LCD_MOSI), miso=Pin(LCD_MISO))

dc = Pin(LCD_DC, Pin.OUT)

tft = st7796.ST7796(spi, LCD_W, LCD_H, reset=None, dc=dc, cs=None, rotation=0, invert=True)

# I2C
i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=400_000)

tca = TCA9554(i2c, TCA_ADDR)

# Backlight + reset via TCA9554
# Some revisions also drive BL on GPIO6; this board works with TCA BL.
tca.pin_mode(TCA_PIN_BL, True)
tca.pin_mode(TCA_PIN_RST, True)
tca.write_pin(TCA_PIN_BL, 1)

tca.write_pin(TCA_PIN_RST, 1)
time.sleep_ms(10)
tca.write_pin(TCA_PIN_RST, 0)
time.sleep_ms(10)
tca.write_pin(TCA_PIN_RST, 1)
time.sleep_ms(120)

# Touch auto-detect
scan = i2c.scan()
if 0x38 in scan:
    touch = FT6336(i2c, 0x38)
elif 0x5D in scan:
    touch = GT911(i2c, 0x5D)
elif 0x14 in scan:
    touch = GT911(i2c, 0x14)
else:
    touch = None

# SD card mount

def mount_sd():
    if USE_SDMMC:
        sd = SDCard(slot=1, width=1, sck=Pin(SD_CLK), cmd=Pin(SD_CMD), data=(Pin(SD_D0),))
        os.mount(sd, "/sd")
        return sd
    else:
        if sdcard is None:
            raise RuntimeError("sdcard.py not found for SPI mode")
        spi_sd = SPI(1, baudrate=20_000_000, polarity=0, phase=0,
                     sck=Pin(SD_SCK), mosi=Pin(SD_MOSI), miso=Pin(SD_MISO))
        sd = sdcard.SDCard(spi_sd, Pin(SD_CS))
        os.mount(sd, "/sd")
        return sd

try:
    sd = mount_sd()
except Exception:
    sd = None

# ---- UI ----
BLACK = 0x0000
WHITE = 0xFFFF
RED   = 0xF800
GREEN = 0x07E0
BLUE  = 0x001F
YELLOW= 0xFFE0
GRAY  = 0x7BEF
NAVY  = 0x0010

ROW_H = 28
LIST_TOP = 40
LIST_BOTTOM = 400
BTN_H = 40

BTN_RUN = (10, 430, 140, BTN_H)
BTN_RST = (170, 430, 140, BTN_H)


def draw_button(x, y, w, h, label, bg, fg):
    tft.fill_rect(x, y, w, h, bg)
    tft.rect(x, y, w, h, WHITE)
    tft.text(label, x + 10, y + 12, fg)


def get_py_files():
    try:
        return sorted([f for f in os.listdir("/sd") if f.endswith(".py")])
    except Exception:
        return []


def draw_menu(files, selected):
    tft.fill(NAVY)
    tft.text("Tool Launcher", 10, 10, YELLOW)

    if not files:
        tft.text("No .py files in /sd", 10, 60, WHITE)
    else:
        max_rows = (LIST_BOTTOM - LIST_TOP) // ROW_H
        start = 0
        end = min(len(files), start + max_rows)
        for i in range(start, end):
            y = LIST_TOP + (i - start) * ROW_H
            if i == selected:
                tft.fill_rect(5, y, LCD_W - 10, ROW_H, BLUE)
            tft.text(files[i][:20], 10, y + 8, WHITE)

    draw_button(*BTN_RUN, "Run", GREEN, BLACK)
    draw_button(*BTN_RST, "Reset", RED, WHITE)
    tft.show()


def in_rect(x, y, rect):
    rx, ry, rw, rh = rect
    return (rx <= x < rx + rw) and (ry <= y < ry + rh)


def run_script(path):
    tft.fill(BLACK)
    tft.text("Running:", 10, 10, WHITE)
    tft.text(path[-20:], 10, 30, WHITE)
    tft.show()
    try:
        with open(path, "r") as f:
            code = f.read()
        g = {"__name__": "__main__", "__file__": path}
        exec(code, g)
    except Exception as e:
        tft.fill(BLACK)
        tft.text("Error:", 10, 10, RED)
        tft.text(str(e)[:26], 10, 30, WHITE)
        tft.show()
        time.sleep_ms(1500)


files = get_py_files()
selected = 0 if files else -1
draw_menu(files, selected)

last_touch = False

while True:
    p = touch.get_point() if touch else None
    pt = map_touch(p)

    if pt and not last_touch:
        x, y = pt
        if in_rect(x, y, BTN_RST):
            reset()
        elif in_rect(x, y, BTN_RUN) and selected >= 0:
            run_script("/sd/" + files[selected])
            gc.collect()
            files = get_py_files()
            selected = 0 if files else -1
            draw_menu(files, selected)
        elif LIST_TOP <= y < LIST_BOTTOM and files:
            idx = (y - LIST_TOP) // ROW_H
            if idx < len(files):
                selected = idx
                draw_menu(files, selected)
                run_script("/sd/" + files[selected])
                gc.collect()
                files = get_py_files()
                selected = 0 if files else -1
                draw_menu(files, selected)
        last_touch = True
    elif not pt:
        last_touch = False

    time.sleep_ms(30)
