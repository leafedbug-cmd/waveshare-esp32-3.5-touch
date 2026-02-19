# Minimal ST7796 driver for MicroPython (RGB565 framebuf)
import time
import framebuf

class ST7796:
    def __init__(self, spi, width, height, reset=None, dc=None, cs=None, rotation=0, invert=True):
        self.spi = spi
        self.width = width
        self.height = height
        self.reset = reset
        self.dc = dc
        self.cs = cs
        self.rotation = rotation & 3
        self.invert = invert

        self.buffer = bytearray(self.width * self.height * 2)
        self.fb = framebuf.FrameBuffer(self.buffer, self.width, self.height, framebuf.RGB565)

        if self.cs is not None:
            self.cs.init(self.cs.OUT, value=1)
        if self.dc is not None:
            self.dc.init(self.dc.OUT, value=0)
        if self.reset is not None:
            self.reset.init(self.reset.OUT, value=1)

        self.init()

    def _cs_low(self):
        if self.cs is not None:
            self.cs.value(0)

    def _cs_high(self):
        if self.cs is not None:
            self.cs.value(1)

    def _dc_low(self):
        if self.dc is not None:
            self.dc.value(0)

    def _dc_high(self):
        if self.dc is not None:
            self.dc.value(1)

    def write_cmd(self, cmd):
        self._dc_low()
        self._cs_low()
        self.spi.write(bytearray([cmd]))
        self._cs_high()

    def write_data(self, data):
        self._dc_high()
        self._cs_low()
        self.spi.write(data)
        self._cs_high()

    def hw_reset(self):
        if self.reset is None:
            return
        self.reset.value(1)
        time.sleep_ms(10)
        self.reset.value(0)
        time.sleep_ms(10)
        self.reset.value(1)
        time.sleep_ms(120)

    def init(self):
        self.hw_reset()
        self.write_cmd(0x01)  # SWRESET
        time.sleep_ms(120)
        self.write_cmd(0x11)  # SLPOUT
        time.sleep_ms(120)

        # 16-bit color
        self.write_cmd(0x3A)
        self.write_data(b"\x55")

        # MADCTL (rotation + BGR)
        madctl = self._madctl_for_rotation(self.rotation)
        self.write_cmd(0x36)
        self.write_data(bytearray([madctl]))

        # Inversion for IPS panels
        if self.invert:
            self.write_cmd(0x21)  # INVON
        else:
            self.write_cmd(0x20)  # INVOFF

        self.write_cmd(0x29)  # DISPON
        time.sleep_ms(20)

    def _madctl_for_rotation(self, rot):
        # MX=0x40, MY=0x80, MV=0x20, BGR=0x08
        if rot == 0:
            return 0x68  # MX | MV | BGR
        if rot == 1:
            return 0xC8  # MX | MY | BGR
        if rot == 2:
            return 0xA8  # MY | MV | BGR
        return 0x08       # BGR

    def set_window(self, x0, y0, x1, y1):
        self.write_cmd(0x2A)  # CASET
        self.write_data(bytearray([x0 >> 8, x0 & 0xFF, x1 >> 8, x1 & 0xFF]))
        self.write_cmd(0x2B)  # RASET
        self.write_data(bytearray([y0 >> 8, y0 & 0xFF, y1 >> 8, y1 & 0xFF]))
        self.write_cmd(0x2C)  # RAMWR

    def show(self):
        self.set_window(0, 0, self.width - 1, self.height - 1)
        self.write_data(self.buffer)

    # Drawing helpers (draw into frame buffer)
    def fill(self, color):
        self.fb.fill(color)

    def fill_rect(self, x, y, w, h, color):
        self.fb.fill_rect(x, y, w, h, color)

    def rect(self, x, y, w, h, color):
        self.fb.rect(x, y, w, h, color)

    def text(self, s, x, y, color):
        self.fb.text(s, x, y, color)
