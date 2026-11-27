#!/usr/bin/env python3
"""OV2659 SCCB, scaled YUYV output and LS26 DVP-to-PSRAM DMA."""
from functools import partial
import time

from qemu_camera_test import CameraMachine
from qemu_test import ROOT
from run_qemu_camera import transfer
from run_qemu_camera_capture import DVP, DMA

OUTPUT = ROOT / 'artifacts/qemu' / ('ov2659-' + time.strftime('%Y%m%d-%H%M%S'))
I2C, PINMUX, GPIO, RAM = 0x46e00000, 0x47500000, 0x46700000, 0x28010000
SENTINEL = 0xa5a5a5a5
PORTS = dict(SCL=33, SDA=32, MCLK=26, PCLK=12, HSYNC=10, VSYNC=11, RESET=21)
PORTS.update({'D%d' % i: 13 + i for i in range(8)})
PLAN = {'version': 1, 'chip': 'ls2684', 'buttons': [], 'indicators': [],
        'camera': {'model': 'ov2659', 'pads': PORTS, 'pwdn_strap': False}}


def setup(m):
    m.write(0x45800008, 0x208000)
    for pin in (32, 33):
        m.write(PINMUX + 4 * pin, 8)
    m.write(GPIO + 0x30, 1 << 21)
    m.write(GPIO + 0x28, 1 << 21)
    for pin in (26, *range(10, 21)):
        m.write(PINMUX + 4 * pin, 16)
    m.write(0x4580001c, 2)
    m.write(DVP + 0x18, 11)
    m.write(DVP + 0x10, 3)
    m.write(DVP + 0x14, 1)
    m.write(DVP + 0x24, 8)
    m.write(DVP + 0x28, 0x13f)
    m.write(I2C + 0x2c, 5 | (200 << 4))
    return partial(transfer, m, address=0x30, bus=I2C, irq=44)


def write_register(xfer, reg, value):
    assert xfer([reg >> 8, reg & 255, value])[0] & 0x400, hex(reg)


def read_register(xfer, reg, count=1):
    assert xfer([reg >> 8, reg & 255], stop=False)[0] & 0x400
    return xfer(receive=count, reset=False)[1]


def mode(xfer, w, h, fast):
    # OV2659 product spec v2.0, tables 2-1, 2-5, 7-5 and 7-10.
    # The two 320x240 readouts use different array binning and PLL settings.
    crop = w != 400
    xoff, yoff = ((88, 62) if fast else (176, 126)) if crop else (8, 2)
    hts, vts = (1300, 616) if fast else (1951, 1232)
    def word(reg, n):
        write_register(xfer, reg, n >> 8)
        write_register(xfer, reg + 1, n & 255)
    for reg, n in [(0x3804, 1631), (0x3806, 1207 if fast else 1211),
                   (0x3808, w), (0x380a, h), (0x380c, hts), (0x380e, vts),
                   (0x3810, xoff), (0x3812, yoff)]:
        word(reg, n)
    for reg, n in [(0x3000, 3), (0x3001, 0xfc), (0x3002, 0xe0),
                   (0x3004, 0x20), (0x3005, 40 if fast else 36),
                   (0x3814, 0x11 if w == 1280 else 0x31),
                   (0x3815, 0x11 if w == 1280 else 0x31),
                   (0x3820, 0x80 | fast), (0x3821, int(fast)),
                   (0x4300, 0x30), (0x5002, 0 if w == 1280 else 16),
                   (0x4702, 2), (0x0100, 1)]:
        write_register(xfer, reg, n)
    assert read_register(xfer, 0x3808, 4) == [w >> 8, w & 255, h >> 8, h & 255]


def arm(m, w, h):
    m.write(DVP, w)
    m.write(DVP + 4, h)
    m.write(DVP + 0x2c, 0x7ff)
    m.write(DMA + 0x154, 0xfff)
    m.write(DMA + 0x54, 0x45001000)
    m.write(DMA + 0x5c, RAM)
    m.write(DMA + 0x270, 2)
    m.write(DMA + 0x2c, w * h // 2)
    m.write(DMA + 0x28, 17)
    m.write(DMA, 0x5000c283)
    m.write(DVP + 0x20, 1)


def capture(w, h, fast):
    m = CameraMachine(OUTPUT / ('%dx%d-%s' % (w, h, fast)),
                      board='arcs-board', assembly=PLAN, budget_ns=3000000000)
    try:
        xfer = setup(m)
        assert read_register(xfer, 0x300a, 2) == [0x26, 0x56]
        assert not xfer([0x30, 0x0a, 0])[0] & 0x400
        assert not xfer([0xf1, 0x00])[0] & 0x400
        m.write(DVP + 0x10, 2)
        assert not xfer()[0] & 0x400
        m.write(DVP + 0x10, 3)
        write_register(xfer, 0x0103, 1)
        assert read_register(xfer, 0x0100) == [0]
        mode(xfer, w, h, fast)
        rotated = w == 320 and fast
        if rotated:
            write_register(xfer, 0x3820, 0x87)
            write_register(xfer, 0x3821, 0x07)
        last = RAM + 2*w*h
        for addr in (RAM - 4, RAM, last - 4, last):
            m.write(addr, SENTINEL)
        arm(m, w, h)
        m.command('clock_step 500000000')
        assert not m.read(DVP + 0x38) and m.read(RAM) == SENTINEL
        # A grayscale ramp exercises each byte of each YUYV pair. Crop and
        # mirror affect coordinates; neutral chroma must remain at 128.
        cropped = w != 400
        rgb = bytearray(640*480*3)
        for y in range(480):
            for x in range(640):
                shade = (x + y) % 256
                start = 3*(640*y + x)
                rgb[start:start+3] = bytes([shade])*3
        m.camera_frame(bytes(rgb))
        m.command('clock_step 1')
        assert m.read(DVP + 0x38) & 0x80
        m.write(DVP + 0x2c, 0x80)
        # Replace pending input during the active frame.
        m.camera_frame(bytes([255, 0, 0]) * (640 * 480))
        # 12.5 MHz XVCLK; allow one active interval, before next SOF.
        m.command('clock_step %d' % (70000000 if fast else 215000000))
        assert m.read(DMA + 0x158) & 1
        assert m.read(DVP + 0x38) & 0x48 == 0x40
        def luma(x, y):
            if rotated:
                x, y = w - 1 - x, h - 1 - y
            sx = 64 + x*512//w if cropped else x*640//w
            sy = 48 + y*384//h if cropped else y*480//h
            return (sx + sy) % 256
        for y in (0, h//2, h-1):
            for x in (0, w//2, w-2):
                expected = 0x80008000 | (luma(x+1,y) << 16) | luma(x,y)
                assert m.read(RAM + 2*(y*w+x)) == expected
        assert m.read(RAM-4) == m.read(last) == SENTINEL
        m.write(DVP + 0x20, 0)
        # Stream stop and RESET cancel future DMA. Color bars are internal
        # sensor pixels and can then be selected explicitly with no scene.
        m.camera_frame(None)
        write_register(xfer, 0x0100, 0)
        arm(m, 16, 1)
        m.command('clock_step 2000000')
        assert m.read(DVP + 0x38) == 0
        write_register(xfer, 0x50a0, 0x80)
        write_register(xfer, 0x0100, 1)
        m.command('clock_step 2000000')
        assert m.read(RAM) == (0x80008000 if rotated else 0x80ff80ff)
        assert m.read(DMA + 0x158) & 1
        m.write(DVP + 0x20, 0)
        arm(m, 16, 1)
        m.command('clock_step 1')
        m.write(GPIO + 0x2c, 1 << 21)
        m.command('clock_step 2000000')
        assert not m.read(DMA + 0x158) & 1
        assert not xfer()[0] & 0x400
        m.write(GPIO + 0x30, 1 << 21)
        assert read_register(xfer, 0x0100) == [0]
        print('OV2659 %dx%d %s: SCCB, full YUYV DMA, IRQ, frame latch and reset: PASS' %
              (w, h, 'binned' if fast else 'full array'))
    finally:
        m.close()


def unsupported_format():
    m = CameraMachine(OUTPUT / 'unsupported', board='arcs-board', assembly=PLAN)
    try:
        xfer = setup(m)
        mode(xfer, 400, 300, True)
        write_register(xfer, 0x50a0, 0x80)
        write_register(xfer, 0x4300, 0x00)  # RAW is outside the modeled modes.
        arm(m, 16, 1)
        try:
            m.command('clock_step 1')
        except EOFError:
            assert m.process.wait(timeout=5) == 1
            assert 'Unsupported OV2659 DVP output format' in (m.directory / 'qemu.log').read_text()
        else:
            raise AssertionError('Unsupported OV2659 format produced a frame')
    finally:
        m.close()


if __name__ == '__main__':
    for settings in [(400, 300, True), (320, 240, False),
                     (320, 240, True), (1280, 960, False)]:
        capture(*settings)
    unsupported_format()
