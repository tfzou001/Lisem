#!/usr/bin/env python3
"""JD9853 scanout and BL6138 flash/point protocol through LS26 pad routes."""
import time

from qemu_test import Machine, ROOT
from run_qemu_display import command, step
from run_qemu_sc919 import pixel

OUTPUT = ROOT / 'artifacts/qemu' / ('jd9853-bl6138-' + time.strftime('%Y%m%d-%H%M%S'))
GPIO = 0x46700000
I2C = 0x46e00000
MUX = 0x47500000
RESET = 1 << 4
INT = 1 << 27
PLAN = {
    'version': 1, 'chip': 'ls2684', 'buttons': [], 'indicators': [],
    'panel': {
        'model': 'jd9853', 'id': 'screen', 'controller_id': 'lcd', 'backlight_id': 'light',
        'pads': {'SCK': 25, 'SDI': 24, 'CS': 22, 'DC': 23, 'RESET': 9, 'TE': 8},
        'width': 240, 'height': 320, 'x_offset': 0, 'y_offset': 0, 'rotation': 180,
        'inverted': False, 'backlight_pad': 0, 'backlight_active': 1,
    },
    'touch': {'model': 'bl6138', 'SCL': 33, 'SDA': 32, 'RESET': 4, 'INT': 27},
}


def configure(m):
    for pad in (22, 24, 25):
        m.write(MUX + pad * 4, 5)
    for pad in (32, 33):
        m.write(MUX + pad * 4, 8)
    m.write(GPIO + 0x30, 1 | RESET | (1 << 9))
    m.write(GPIO + 0x28, 1 | RESET | (1 << 9) | (1 << 23))
    m.write(I2C + 0x2c, 5 | (20 << 4))
    step(m, 120000000)


def display(m):
    command(m, 1)
    step(m, 120000000)
    for op, data in [(0xdf, [0x98, 0x53]), (0xde, [0]), (0xb2, [0x20]),
                     (0xc4, list(range(12))), (0xde, [1]), (0xc4, [0x72, 0x12]),
                     (0xde, [0]), (0x35, [0]), (0x36, [0xd0]), (0x3a, [0x55])]:
        command(m, op, data)
    m.write(GPIO + 0x58, 5)  # PA8 falling-edge TE
    m.write(GPIO + 0x50, 1 << 8)
    command(m, 0x11)
    step(m, 6000000)
    command(m, 0x29)  # 5 ms command recovery; 120 ms applies to SLPIN.
    step(m, 120000000)
    assert m.read(GPIO + 0x64) & (1 << 8)
    corners = [(0, 0, 0xf800, b'\xff\0\0'), (239, 0, 0x07e0, b'\0\xff\0'),
               (0, 319, 0x001f, b'\0\0\xff'), (239, 319, 0xffff, b'\xff\xff\xff')]
    for x, y, rgb565, _ in corners:
        command(m, 0x2a, list(x.to_bytes(2, 'big')) * 2)
        command(m, 0x2b, list(y.to_bytes(2, 'big')) * 2)
        command(m, 0x2c, list(rgb565.to_bytes(2, 'big')))
    step(m, 40000000)
    path = m.directory / 'mounted-corners.ppm'
    m.qmp_command('screendump', {'filename': str(path), 'format': 'ppm'})
    header = b'P6\n240 320\n255\n'
    frame = path.read_bytes()
    assert frame.startswith(header)
    for x, y, _, rgb in corners:
        offset = len(header) + 3 * (y * 240 + x)
        assert frame[offset:offset + 3] == rgb
    command(m, 0x2a, [0, 0, 0, 0])
    command(m, 0x2b, [0, 0, 0, 0])
    m.write(MUX + 24 * 4, 0)
    command(m, 0x2c, [0, 0x1f])
    assert pixel(m, 'unrouted') == bytes([255, 0, 0])
    m.write(MUX + 24 * 4, 5)
    m.write(MUX, 12)
    m.write(0x47300044, (59 << 16) | 39)
    m.write(0x47300024, 3 | (1 << 19))
    assert pixel(m, 'pwm-active-high') == bytes([153, 0, 0])
    command(m, 0x10)
    assert pixel(m, 'sleep') == bytes(3)


def transfer(m, data=(), count=0, stop=True, reset=True):
    if reset:
        m.write(I2C + 0x28, 5)
    else:
        m.write(I2C + 0x18, 0x3f8)
    m.write(I2C + 0x1c, 0x2c)
    m.write(I2C + 0x24, 0x1c00 | (0x200 if stop else 0) |
            (0x100 if count else 0) | (count or len(data)))
    sent, received = 0, bytearray()
    m.write(I2C + 0x28, 1)
    for _ in range(1000):
        status = m.read(I2C + 0x18)
        if count:
            while not m.read(I2C + 0x18) & 1:
                received.append(m.read(I2C + 0x20))
        else:
            while sent < len(data) and not m.read(I2C + 0x18) & 2:
                m.write(I2C + 0x20, data[sent])
                sent += 1
        if status & 0x200:
            return bool(status & (8 if count else 0x400)), bytes(received)
        step(m, 10000)
    raise AssertionError('BL6138 I2C transfer deadline')


def write(m, *data):
    assert transfer(m, data)[0], bytes(data).hex()


def read(m, register, count):
    assert transfer(m, [register], stop=False)[0]
    ack, result = transfer(m, count=count, reset=False)
    assert ack and len(result) == count
    return result


def enter(m):
    m.write(GPIO + 0x2c, INT)
    m.write(GPIO + 0x28, m.read(GPIO + 0x28) | INT)
    write(m, 0x0a, 0xf5, 0xe7, 0xe8)
    # Firmware waits nominally 50 RTOS ticks; its first tick can be partial.
    step(m, 49000000)


def leave(m):
    m.write(GPIO + 0x30, INT)
    m.write(GPIO + 0x28, m.read(GPIO + 0x28) & ~INT)
    m.write(GPIO + 0x2c, RESET)
    step(m, 20000000)
    m.write(GPIO + 0x30, RESET)
    step(m, 119000000)
    write(m, 0x55, 0x46, 0x4f)


def program(m, address, data):
    size = len(data)
    write(m, 0x0f, 0xf0, *address.to_bytes(2, 'big'),
          *(address + size - 1).to_bytes(2, 'big'), *data)
    step(m, 4000000)


def flash_test(m, flash_path):
    assert not transfer(m, [1])[0]
    assert not transfer(m, [0x0a, 0xf5, 0xe7, 0xe8])[0]  # INT not driven low.
    enter(m)
    write(m, 0x0a, 0xf5, 0xe7)
    assert transfer(m, count=1) == (True, bytes([0x23]))
    write(m, 0x09, 0xf6)
    assert not transfer(m, [0x0a, 0xf5, 0xe7])[0]
    step(m, 49000000)
    assert flash_path.read_bytes() == bytes([255]) * 32768
    assert not transfer(m, [0x0f, 0xf0, 0x7f, 0xff, 0x80, 0])[0]
    program(m, 0x200, bytes(range(128)))
    program(m, 0x5fc, bytes.fromhex('aa55'))
    write(m, 0x0d, 0xf2, 5, 0xfc, 5, 0xfd)
    assert transfer(m, count=2) == (True, bytes.fromhex('aa55'))
    # Reset before erase completion. Previously programmed bytes must survive.
    write(m, 0x09, 0xf6)
    leave(m)
    assert read(m, 0xb6, 3) == bytes([42, 43, 44])
    write(m, 0x8a, 0xaa)
    assert read(m, 0x3f, 3) == bytes(3)
    step(m, 250000000)
    checksum = sum(flash_path.read_bytes()) & 0xffff
    assert read(m, 0x3f, 3) == bytes([1]) + checksum.to_bytes(2, 'big')


def points(m):
    def contact(value):
        m.qmp_command('qom-set', {'path': '/machine', 'property': 'x-lisa-touch', 'value': value})
    assert read(m, 1, 6)[2] == 0xc0
    m.write(GPIO + 0x60, 5 << 12)  # Falling edge on PA27.
    m.write(GPIO + 0x50, INT)
    contact('23,301,press')
    assert m.read(GPIO + 0x64) & INT
    assert read(m, 1, 6) == bytes([0, 1, 0, 23, 1, 45])
    assert read(m, 1, 6)[2] == 0x80
    for x, y in ((0, 0), (239, 0), (0, 319), (239, 319)):
        contact(f'{x},{y},press')
        assert read(m, 1, 6) == bytes([0, 1, 0x80, x, y >> 8, y & 255])
    contact('release')
    assert read(m, 1, 6) == bytes([0, 0, 0x40, 239, 1, 63])
    first = read(m, 0xf9, 4)
    step(m, 20000000)
    assert read(m, 0xf9, 4) != first
    assert not transfer(m, [0xce])[0]
    write(m, 0xa5, 3)
    assert not transfer(m, [0x01])[0]


def main():
    backing = OUTPUT / 'bl6138-flash.bin'
    for index in range(2):
        m = Machine(OUTPUT / str(index), board='arcs-board', assembly=PLAN, desktop=True,
                    touch_flash=backing, budget_ns=3000000000)
        try:
            configure(m)
            if index == 0:
                display(m)
                flash_test(m, backing)
            else:
                assert read(m, 0xb6, 3) == bytes([42, 43, 44])
            points(m)
        finally:
            m.close()
    print('JD9853 routed SPI/TE/PWM scanout and BL6138 I2C/flash/IRQ/power cycling: PASS')


if __name__ == '__main__':
    main()
