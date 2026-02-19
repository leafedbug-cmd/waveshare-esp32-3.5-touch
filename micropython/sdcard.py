# SPI SD card driver for MicroPython (based on official MicroPython driver)
from machine import Pin
from micropython import const
import time

_CMD_TIMEOUT = const(100)
_TOKEN_DATA = const(0xFE)

R1_IDLE_STATE = const(1 << 0)
R1_ILLEGAL_COMMAND = const(1 << 2)

class SDCard:
    def __init__(self, spi, cs):
        self.spi = spi
        self.cs = cs
        self.cs.init(self.cs.OUT, value=1)
        self.cmdbuf = bytearray(6)
        self.tokenbuf = bytearray(16)
        self.cdv = 512
        self.sectors = 0
        self.init_card()

    def init_card(self):
        # init SPI at low speed
        self.spi.init(baudrate=100_000, polarity=0, phase=0)
        for _ in range(16):
            self.spi.write(b"\xFF")

        # CMD0: go idle
        if self.cmd(0, 0, 0x95) != R1_IDLE_STATE:
            raise OSError("no SD card")

        # CMD8: check SD version
        r = self.cmd(8, 0x01AA, 0x87, 4)
        if r == R1_IDLE_STATE:
            # SD v2
            if self.tokenbuf[2] != 0x01 or self.tokenbuf[3] != 0xAA:
                raise OSError("bad SD card")
            # ACMD41 with HCS
            for _ in range(5000):
                self.cmd(55, 0, 0)
                if self.cmd(41, 0x40000000, 0) == 0:
                    break
            else:
                raise OSError("timeout waiting for card")
            # CMD58 to read OCR
            if self.cmd(58, 0, 0, 4) != 0:
                raise OSError("CMD58 failed")
            if self.tokenbuf[0] & 0x40:
                self.cdv = 1
        elif r == (R1_IDLE_STATE | R1_ILLEGAL_COMMAND):
            # SD v1
            for _ in range(5000):
                self.cmd(55, 0, 0)
                if self.cmd(41, 0, 0) == 0:
                    break
            else:
                raise OSError("timeout waiting for card")
        else:
            raise OSError("could not init card")

        # CMD16: set block length
        if self.cmd(16, 512, 0) != 0:
            raise OSError("CMD16 failed")

        # read CSD to get size
        self.sectors = self._read_csd_sectors()

        # set high speed
        self.spi.init(baudrate=20_000_000, polarity=0, phase=0)

    def cmd(self, cmd, arg, crc, final=0, release=True):
        self.cs.value(0)
        self.cmdbuf[0] = 0x40 | cmd
        self.cmdbuf[1] = arg >> 24
        self.cmdbuf[2] = arg >> 16
        self.cmdbuf[3] = arg >> 8
        self.cmdbuf[4] = arg
        self.cmdbuf[5] = crc
        self.spi.write(self.cmdbuf)

        for _ in range(_CMD_TIMEOUT):
            r = self.spi.read(1, 0xFF)[0]
            if r & 0x80 == 0:
                break
        else:
            r = 0xFF

        for i in range(final):
            self.tokenbuf[i] = self.spi.read(1, 0xFF)[0]

        if release:
            self.cs.value(1)
            self.spi.write(b"\xFF")
        return r

    def _read_csd_sectors(self):
        # CMD9: read CSD
        if self.cmd(9, 0, 0, release=False) != 0:
            self.cs.value(1)
            raise OSError("CMD9 failed")

        # wait for data token
        for _ in range(_CMD_TIMEOUT * 100):
            if self.spi.read(1, 0xFF)[0] == _TOKEN_DATA:
                break
        else:
            self.cs.value(1)
            raise OSError("timeout waiting for CSD")

        csd = bytearray(16)
        self.spi.readinto(csd)
        self.spi.read(2, 0xFF)  # CRC
        self.cs.value(1)
        self.spi.write(b"\xFF")

        csd_structure = (csd[0] >> 6) & 0x03
        if csd_structure == 1:
            # CSD version 2.0
            c_size = ((csd[7] & 0x3F) << 16) | (csd[8] << 8) | csd[9]
            return (c_size + 1) * 1024
        # CSD version 1.0
        c_size = ((csd[6] & 0x03) << 10) | (csd[7] << 2) | ((csd[8] & 0xC0) >> 6)
        c_size_mult = ((csd[9] & 0x03) << 1) | ((csd[10] & 0x80) >> 7)
        read_bl_len = csd[5] & 0x0F
        block_len = 1 << read_bl_len
        mult = 1 << (c_size_mult + 2)
        blocknr = (c_size + 1) * mult
        return blocknr * (block_len // 512)

    def readblocks(self, block_num, buf):
        if len(buf) % 512 != 0:
            raise OSError("buf len not multiple of 512")
        self.cs.value(0)
        if self.cmd(17, block_num * self.cdv, 0, release=False) != 0:
            self.cs.value(1)
            raise OSError("CMD17 failed")

        for _ in range(_CMD_TIMEOUT * 100):
            if self.spi.read(1, 0xFF)[0] == _TOKEN_DATA:
                break
        else:
            self.cs.value(1)
            raise OSError("timeout waiting for data")

        self.spi.readinto(buf)
        self.spi.read(2, 0xFF)
        self.cs.value(1)
        self.spi.write(b"\xFF")

    def writeblocks(self, block_num, buf):
        if len(buf) % 512 != 0:
            raise OSError("buf len not multiple of 512")

        self.cs.value(0)
        if self.cmd(24, block_num * self.cdv, 0, release=False) != 0:
            self.cs.value(1)
            raise OSError("CMD24 failed")

        self.spi.write(bytearray([_TOKEN_DATA]))
        self.spi.write(buf)
        self.spi.write(b"\xFF\xFF")

        resp = self.spi.read(1, 0xFF)[0]
        if (resp & 0x1F) != 0x05:
            self.cs.value(1)
            raise OSError("write failed")

        while self.spi.read(1, 0xFF)[0] == 0:
            pass

        self.cs.value(1)
        self.spi.write(b"\xFF")

    def ioctl(self, op, arg):
        if op == 4:  # get number of blocks
            return self.sectors
        if op == 5:  # get block size
            return 512
        if op == 6:  # erase block
            return 0
        if op == 1:  # init
            self.init_card()
            return 0
        if op == 2:  # deinit
            return 0
        return 0
