#!/usr/bin/env python3
"""Exercise the NT26 through the LS26 MUSB host, GPIO and Ethernet bridge."""
import json
import os
from pathlib import Path
import struct
import socket
import sys
import time
import tempfile
from unittest.mock import patch

from qemu_test import Machine, ROOT, write_bytes, read_bytes
from run_host_network_native import checksum, GUEST, HOST, transport

PLAN = {'version': 1, 'chip': 'ls2684', 'buttons': [], 'indicators': [],
        'modem': {'id': 'cellular', 'model': 'lierda-nt26k0b1', 'usb': 0,
                  'pwrkey_inverted': True, 'DTR': 35, 'PWRKEY': 38, 'RI': 41}}
MUSB = 0x41000000
GPIOB = 0x46800000


class Host:
    def __init__(self, m):
        self.m = m

    def read(self, reg, width='b'):
        return self.m.command(f'read{width} {MUSB + reg:#x}')

    def write(self, reg, value, width='b'):
        self.m.command(f'write{width} {MUSB + reg:#x} {value}')

    def step(self, ns=125000):
        self.m.command(f'clock_step {ns}')

    def fifo_write(self, ep, data):
        for pos in range(0, len(data) - len(data) % 4, 4):
            self.write(0x20 + 4 * ep, int.from_bytes(data[pos:pos+4], 'little'), 'l')
        for b in data[len(data) - len(data) % 4:]:
            self.write(0x20 + 4 * ep, b)

    def fifo_read(self, ep, count):
        data = bytearray()
        while len(data) + 4 <= count:
            data.extend(self.read(0x20 + 4 * ep, 'l').to_bytes(4, 'little'))
        while len(data) < count:
            data.append(self.read(0x20 + 4 * ep))
        return bytes(data)

    def status(self):
        return json.loads(self.m.qmp_command('qom-get', {
            'path': '/machine', 'property': 'x-lisem-modem-status'}))

    def change(self, key, value, generation=None, error=False):
        if generation is None:
            generation = self.status()['usb_generation']
        return self.m.qmp_command('qom-set', {'path': '/machine', 'property': 'x-lisem-nt26-control',
            'value': json.dumps({'usb_generation': generation, key: value})}, error=error)

    def control(self, kind, request, value, index=0, length=0):
        self.fifo_write(0, struct.pack('<BBHHH', kind, request, value, index, length))
        self.write(0x102, 10)
        self.step()
        assert self.read(2, 'w') & 1 and self.read(0x102) == 0
        out = bytearray()
        if kind & 128:
            while len(out) < length:
                self.write(0x102, 0x20)
                self.step()
                assert self.read(2, 'w') & 1 and self.read(0x102) == 1
                count = self.read(0x108, 'w')
                out.extend(self.fifo_read(0, count))
                self.write(0x102, 0)
                if count < 64:
                    break
        self.write(0x102, 0x42 if kind & 128 else 0x60)
        self.step()
        assert self.read(2, 'w') & 1
        assert self.read(0x102) == (0 if kind & 128 else 1)
        self.write(0x102, 0)
        return bytes(out)

    def out(self, target, payload):
        for pos in range(0, len(payload) + 1, 512):
            self.write(0x110, 512, 'w')
            self.write(0x11a, 0x60 | target)
            self.fifo_write(1, payload[pos:pos+512])
            self.write(0x112, 1)
            self.step()
            assert self.read(2, 'w') & 2 and self.read(0x112) & 0x25 == 0

    def inp(self, target, interrupt=False, nak=False):
        self.write(0x124, 16 if interrupt else 512, 'w')
        self.write(0x12c, (0x70 if interrupt else 0x60) | target)
        self.write(0x126, 0x20)
        self.step()
        if nak:
            assert self.read(0x126) == 0x20 and not self.read(4, 'w') & 4
            self.write(0x126, 0)
            return
        deadline = time.monotonic() + 2
        while self.read(0x126) == 0x20 and time.monotonic() < deadline:
            self.step(1000000)
        assert self.read(4, 'w') & 4 and self.read(0x126) == 1
        result = self.fifo_read(2, self.read(0x128, 'w'))
        self.write(0x126, 0)
        return result

    def at_dma(self, command):
        data = command.encode() + b'\r'
        buf = 0x20050000
        write_bytes(self.m, buf, data)
        self.write(0x110, 512, 'w')
        self.write(0x11a, 0x63)
        self.write(0x113, 0xb4)
        self.write(0x208, buf, 'l')
        self.write(0x20c, len(data), 'l')
        self.write(0x204, 0x1f, 'w')
        self.step()
        assert self.read(0x200, 'l') == 1
        assert self.read(0x208, 'l') == buf + len(data)
        assert self.read(0x20c, 'l') == 0
        self.write(0x113, 0x20)
        self.write(0x112, 1)
        self.step()
        assert self.read(2, 'w') & 2
        return self.inp(3)

    def frame_dma(self):
        # Mode 1 consumes complete 512-byte packets and requests the next
        # token. A short packet remains in the FIFO for the endpoint IRQ.
        buf = 0x20050000
        self.write(0x124, 512, 'w')
        self.write(0x12c, 0x62)
        self.write(0x127, 0xe8)
        self.write(0x208, buf, 'l')
        self.write(0x20c, 1514, 'l')
        self.write(0x204, 0x2d, 'w')
        self.write(0x126, 0x20)
        deadline = time.monotonic() + 2
        while self.read(0x208, 'l') == buf and time.monotonic() < deadline:
            self.step()
        assert self.read(0x208, 'l') == buf + 512, (self.read(0x204, 'w'), self.read(0x126))
        assert not self.read(4, 'w') & 4
        self.step()
        assert self.read(4, 'w') & 4 and self.read(0x126) == 1
        assert self.read(0x20c, 'l') == 1514 - 512
        self.write(0x204, 0, 'w')
        self.write(0x127, 0)
        frame = read_bytes(self.m, buf, 512) + self.fifo_read(2, self.read(0x128, 'w'))
        self.write(0x126, 0)
        return frame

    def frame(self):
        frame = bytearray()
        for _ in range(3):
            chunk = self.inp(2)
            frame.extend(chunk)
            if len(chunk) < 512:
                return bytes(frame)
        raise AssertionError('Unterminated ECM frame')

    def enumerate(self):
        self.write(1, 0x28)
        self.step(20000000)
        self.write(1, 0x20)
        dev = self.control(128, 6, 0x100, length=18)
        assert struct.unpack_from('<HH', dev, 8) == (0x3505, 3)
        self.control(0, 5, 2)
        self.write(0, 2)
        head = self.control(128, 6, 0x200, length=9)
        config = self.control(128, 6, 0x200, length=int.from_bytes(head[2:4], 'little'))
        assert bytes([9, 4, 3, 0, 2, 255, 0, 0, 0]) in config
        self.control(0, 9, 1)
        self.control(1, 11, 1, 1)
        self.control(0x21, 0x43, 0x0e)
        assert self.status()['usb_configured']

    def at(self, command):
        self.out(3, command.encode() + b'\r')
        return self.inp(3)


def ethernet(u):
    mac_string = u.control(128, 6, 0x303, 0x409, 64)[2:].decode('utf-16-le')
    mac = bytes.fromhex(mac_string)
    broadcast = bytes([255]) * 4

    def ip(protocol, source, dest, payload):
        head = bytearray(struct.pack('!BBHHHBBH4s4s', 0x45, 0, 20 + len(payload),
                                     1, 0, 64, protocol, 0, source, dest))
        struct.pack_into('!H', head, 10, checksum(head))
        return head + payload

    def udp_frame(dst_mac, source, dest, sport, dport, payload):
        body = bytearray(struct.pack('!HHHH', sport, dport, 8 + len(payload), 0) + payload)
        pseudo = source + dest + struct.pack('!BBH', 0, 17, len(body))
        struct.pack_into('!H', body, 6, checksum(pseudo + body) or 65535)
        return dst_mac + mac + bytes.fromhex('0800') + ip(17, source, dest, body)

    for msg, expected in [(1, 2), (3, 5)]:
        boot = bytearray(240)
        boot[:8] = bytes.fromhex('0101060026330042')
        boot[28:34] = mac
        boot[236:240] = bytes.fromhex('63825363')
        boot += bytes([53, 1, msg, 61, 7, 1]) + mac
        if msg == 3:
            boot += bytes([50, 4]) + GUEST + bytes([54, 4]) + HOST
        boot += bytes([255])
        u.out(2, udp_frame(bytes([255])*6, bytes(4), broadcast, 68, 67, boot))
        reply = u.frame()
        assert reply[16+42:20+42] == GUEST and bytes([53, 1, expected]) in reply[42+240:], (msg, reply.hex())

    arp = struct.pack('!HHBBH6s4s6s4s', 1, 0x800, 6, 4, 1, mac, GUEST, bytes(6), HOST)
    u.out(2, bytes([255])*6 + mac + bytes.fromhex('0806') + arp)
    reply = u.frame()
    assert reply[12:14] == bytes.fromhex('0806') and reply[20:22] == bytes([0, 2])
    assert reply[28:32] == HOST and reply[32:38] == mac
    gateway = reply[6:12]
    payload = b'NT26 ECM' + bytes(range(256))*2
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server:
        server.bind(('127.0.0.1', 0))
        server.settimeout(2)
        port = server.getsockname()[1]
        u.out(2, udp_frame(gateway, GUEST, HOST, 41001, port, payload))
        data, peer = server.recvfrom(1000)
        assert data == payload
        server.sendto(data[::-1], peer)
        reply = u.frame_dma()
        assert transport(reply, 17, port)[8:] == payload[::-1]

    def tcp(port, seq, ack, flags, data=b''):
        body = bytearray(struct.pack('!HHIIBBHHH', 41002, port, seq, ack, 0x50, flags, 4096, 0, 0))
        body += data
        pseudo = GUEST + HOST + struct.pack('!BBH', 0, 6, len(body))
        struct.pack_into('!H', body, 16, checksum(pseudo + body))
        u.out(2, gateway + mac + bytes.fromhex('0800') + ip(6, GUEST, HOST, body))

    with socket.socket() as server:
        server.bind(('127.0.0.1', 0))
        server.listen()
        server.settimeout(2)
        port = server.getsockname()[1]
        tcp(port, 100, 0, 2)
        synack = transport(u.frame(), 6, port)
        assert synack[13] & 0x12 == 0x12 and int.from_bytes(synack[8:12], 'big') == 101
        remote = int.from_bytes(synack[4:8], 'big') + 1
        tcp(port, 101, remote, 16)
        with server.accept()[0] as conn:
            conn.settimeout(2)
            tcp(port, 101, remote, 24, payload[:128])
            assert conn.recv(1024) == payload[:128]
            conn.sendall(payload[:128][::-1])
            for _ in range(5):
                body = transport(u.frame(), 6, port)
                received = body[(body[12] >> 4) * 4:]
                if received:
                    assert received == payload[:128][::-1]
                    break
            else:
                raise AssertionError('Missing TCP echo over ECM')
    assert u.status()['ecm_errors'] == 0 and u.status()['bridge_drops'] == 0


def run(path, uplink):
    suffix = 'dylib' if sys.platform == 'darwin' else 'so'
    library = ROOT / '.tools/network' / ('libarcs_slirp.' + suffix)
    with patch.dict(os.environ, LISEM_MODEM_NETWORK='1', LISEM_MODEM_NETWORK_LIBRARY=str(library)):
        m = Machine(path, board='arcs-board', assembly=PLAN, network=uplink, loopback=uplink,
                    budget_ns=10000000000, network_capture=False)
    u = Host(m)
    try:
        # USB0 is occupied by the board modem throughout enumeration.
        for bridge, action in [('lab', 'connect'), ('adb', 'enable'), ('cdc', 'enable')]:
            error = m.qmp_command('qom-set', {'path': '/machine/soc',
                'property': f'x-lisem-usb-{bridge}', 'value': action}, error=True)
            assert 'wired to a board device' in error['desc'], error
        m.write(GPIOB + 0x28, (1 << 3) | (1 << 6))
        m.write(GPIOB + 0x2c, 1 << 6)
        # Release firmware keeps PB6 asserted using AON FSEL=2, OEN/OUT force.
        m.write(0x48100018, 0x01c00002)
        u.write(1, 0x20)
        u.write(0x60, 1)
        u.step(999000000)
        assert not u.status()['usb_attached']
        u.step(1000000)
        assert u.status()['usb_attached'] and u.read(10) & 0x10
        u.enumerate()
        assert b'OK' in u.at('ATE0') and b'READY' in u.at('AT+CPIN?')
        assert b'"usbnet",1' in u.at('AT+QCFG="usbnet"')
        assert b'ERROR' in u.at('AT+MISSING')
        assert b'+QCFG: "nat",1' in u.at_dma('AT+QCFG="nat"')
        assert b'OK' in u.at('AT+QCFG="risignaltype","physical"')
        assert b'OK' in u.at('AT+QCFG="urc/ri/other","pulse",20,1')
        assert b'OK' in u.at('AT+QCFG="urc/delay",5')
        assert not u.status()['network_enabled']
        if uplink:
            u.change('network_enabled', True)
        assert b'OK' in u.at('AT+QNETDEVCTL=3,1,1')
        assert u.inp(1, interrupt=True) == bytes([0xa1, 0, 1, 0, 0, 0, 0, 0])
        assert not m.read(GPIOB + 0x20) & (1 << 9)
        u.step(5000000)
        assert u.inp(3) == b'\r\n+QNETDEVSTATUS: 1\r\n'
        u.step(15000000)
        assert m.read(GPIOB + 0x20) & (1 << 9)
        if uplink:
            ethernet(u)
            u.change('network_enabled', False)
            u.inp(2, nak=True)
            u.change('network_enabled', True)
            assert u.status()['network_enabled']
        else:
            u.inp(2, nak=True)
        u.change('sim_present', False)
        old = u.status()['usb_generation']
        assert b'+CME ERROR: 10' in u.at('AT+CPIN?')
        assert b'ERROR' in u.at('AT+QNETDEVCTL=3,1,1')
        assert not u.status()['pdp_active']
        assert b'OK' in u.at('AT+CFUN=1,1')
        u.step(100000000)
        assert not u.status()['usb_attached'] and u.read(10) & 0x20
        u.step(1000000000)
        assert u.read(10) & 0x10
        u.enumerate()
        assert u.change('sim_present', True, old, error=True)
        assert b'+CME ERROR: 10' in u.at('AT+CPIN?')
        assert b'OK' in u.at('AT+QSCLK=1')
        m.write(0x4810000c, 0x01c00002)
        assert not u.status()['usb_attached'] and u.read(10) & 0x20
        m.write(0x4810000c, 0x01400002)
        assert u.status()['usb_attached'] and u.read(10) & 0x10
        u.enumerate()
        assert b'OK' in u.at('AT+QSCLK=0')
    finally:
        m.close()


if __name__ == '__main__':
    out = ROOT / 'artifacts/qemu'
    out.mkdir(parents=True, exist_ok=True)
    tmp = Path(tempfile.mkdtemp(prefix='nt26-', dir=out))
    for enabled in [False, True]:
        run(tmp / str(enabled), enabled)
    print('NT26 GPIO power/DTR/RI, USB PIO/DMA, AT/SIM, ECM DHCP/ARP and real UDP/TCP: PASS')
