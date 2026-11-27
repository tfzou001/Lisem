#!/usr/bin/env python3
"""Run the HK626 V1.2 dual-network application from a packaged Lisem CLI."""
import argparse
import base64
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, HTTPServer
from ipaddress import IPv4Address
import json
import math
from pathlib import Path
import re
import struct
import subprocess
import sys
import threading
import time
import uuid
import wave

from native_service import DEFAULT_BINARY, NativeService, ROOT
from run_mcp import Client

sys.path.insert(0, str(ROOT / 'tools'))
from lpk import FLASH_SIZE, apply_layout, read_lpk
sys.path.insert(0, str(ROOT / 'python/src'))
from lisem.display import DisplayFrame


def check_http(shell, host):
    token = uuid.uuid4().hex
    received = threading.Event()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path != '/' + token:
                self.send_error(404)
                return
            body = token.encode()
            self.send_response(200)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            received.set()

    with HTTPServer((str(host), 0), Handler) as server:
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        try:
            shell(f'http_test http://{host}:{server.server_port}/{token}',
                  rb'http_test success[^\r\n]*response=32 bytes')
            assert received.wait(2), 'Firmware request did not reach the test server'
        finally:
            server.shutdown()
            worker.join()


@contextmanager
def chart_frame(call, destination, *, review):
    capture = call('display_capture', action='start', capacity=160, max_ms=15000)['capture']
    try:
        yield
        last = -1
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            state = call('display_capture', action='status', capture=capture)
            assert state['status'] == 'running', f"Chart was not displayed: {state['status']}"
            index = state['frames'] - 1
            if index == last:
                time.sleep(.05)
                continue
            last = index
            frame = call('display_capture', action='read', capture=capture, index=index,
                         offset=0, limit=1024 * 1024)
            pixels = base64.b64decode(frame['data'], validate=True)
            assert (state['width'], state['height'], len(pixels)) == (240, 320, 240 * 320 * 4)

            def rgb(x, y):
                offset = 4 * (y * 240 + x)
                return pixels[offset:offset + 3][::-1]

            # Each view rotates the chart. The confirmation screen translates
            # its white center line to the right of the preview position.
            bar = rgb(60, 235)
            center = rgb(138, 235)
            if (bar[0] > 75 and max(bar[1:]) < 20 and
                    ((min(center) > 75) if review else center == bar)):
                metadata = {key: state[key] for key in ('run', 'capture', 'width', 'height')}
                metadata.update(frame=frame['frame'], index=index, pixel_format='bgra8')
                DisplayFrame(pixels, metadata).save(destination)
                break
        else:
            raise AssertionError(f'Chart not published for {destination.name}')
    finally:
        call('display_capture', action='discard', capture=capture)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lpk', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--binary', default=DEFAULT_BINARY, type=Path)
    parser.add_argument('--http-host', type=IPv4Address,
                        help='Host interface IPv4 address reachable from both guest networks')
    parser.add_argument('--photo', action='store_true',
                        help='Use the online photo-question screen to capture a test chart')
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    library = output / 'library'
    binary = args.binary.resolve()

    def cli(*arguments):
        reply = subprocess.run([str(binary), '--data-dir', str(library), '--json', *arguments],
                               text=True, capture_output=True, timeout=45)
        assert reply.returncode == 0, reply.stderr
        return json.loads(reply.stdout)

    item = cli('create', '--board', 'snap-learn', '--lpk', str(args.lpk.resolve()))
    device = item['id']
    path = Path(item['path'])
    otp = (path / 'otp.bin').read_bytes()
    assert otp[8:16].hex() == item['uid']
    images = read_lpk(args.lpk)
    assert (path / 'flash.bin').read_bytes() == apply_layout(bytes([255]) * FLASH_SIZE, images)
    touch_image = next(image.data for image in images if image.name == 'tp').ljust(32768, b'\xff')
    checksum = f'{sum(touch_image) & 0xffff:04x}'.encode()
    service = NativeService(library, binary)
    service.created.add(device)
    card = output / 'tf.img'
    # The firmware formats only this newly created disposable image.
    with card.open('xb') as file:
        file.truncate(64 << 20)
    try:
        service.dispatch('sd_card', {'id': device, 'medium': {'path': str(card)}})
        service.dispatch('wifi_environment', {'id': device, 'environment': {'access_points': [
            {'ssid': 'SnapTest', 'rssi': -46, 'channel': 6, 'internet': True}]}})
        if args.photo:
            chart = output / 'camera.ppm'
            bands = [(255, 255, 255), (255, 192, 0), (0, 255, 255), (0, 224, 50),
                     (255, 0, 255), (255, 0, 0), (0, 0, 255), (30, 30, 30)]
            row = b''.join(bytes(color) * 80 for color in bands)
            chart.write_bytes(b'P6\n640 480\n255\n' + row * 232 + b'\xff' * 640 * 3 * 16 + row * 232)
            service.dispatch('camera', {'id': device, 'path': str(chart)})
        for boot in range(3):
            session = service.start(device, online=boot == 1, seconds=120, timeout=300)['session']
            run = session['output']
            uart = Path(run) / 'uart0.bin'

            def call(method, **params):
                return service.dispatch(method, {'id': device, 'run': run, **params})

            def wait(pattern, cursor=0, timeout=90):
                limit = time.monotonic() + timeout
                while time.monotonic() < limit:
                    match = re.search(pattern, uart.read_bytes()[cursor:])
                    if match:
                        return match
                    state = service.dispatch('inspect', {'id': device})['runtime']['session']
                    assert not state['finished'], state.get('error')
                    time.sleep(.1)
                raise AssertionError(f'UART did not match {pattern!r} in {uart}')

            def shell(command, pattern):
                cursor = uart.stat().st_size
                call('uart_write', channel=0, hex=(command + '\r').encode().hex())
                return wait(pattern, cursor)

            def screenshot(filename):
                frame = call('screenshot')
                (output / filename).write_bytes(base64.b64decode(frame['data'], validate=True))

            wait('ui：launcher/page：0'.encode())
            assert b'JD9853 initialized (240x320)' in uart.read_bytes()
            wait(rb'chip=23 fw=\S+ -> \S+ sum=' + checksum + b'/' + checksum + b' attempts=' +
                 (b'1' if boot == 0 else b'0'))
            wait(b'power latched')
            assert (path / 'otp.bin').read_bytes() == otp and service.device(device)['uid'] == item['uid']
            if boot == 0:
                screenshot('home.png')
                shell('mkfs -y', b'format done, filesystem remounted')
                shell('touch_trace record', b'TOUCH_TRACE RECORD START')
                # Deliver both edges to the panel. Firmware samples the BL6138 on I2C1.
                cursor = uart.stat().st_size
                call('touch', x=110, y=297, pressed=True)
                wait(rb'event\S*touch_down', cursor)
                call('touch', x=110, y=297, pressed=False)
                wait(rb'event\S*touch_up', cursor)
                shell('touch_trace stop', b'TOUCH_TRACE STOP OK')
                touch = shell('touch_trace status', rb'TOUCH_TRACE STATUS .*? events=(\d+)')
                assert int(touch[1]) >= 2
                for number, name in enumerate(('ai', 'volume-up', 'back', 'volume-down')):
                    cursor = uart.stat().st_size
                    call('button_sequence', button=name, count=1, hold_ms=120, gap_ms=80)
                    wait(f'adc_button\\[{number}\\]: short_press'.encode(), cursor)
                # Persist the Wi-Fi preference through the original application shell.
                shell('kv set int wifi_onoff 1', b'flash set wifi_onoff:1 success')
                with wave.open(str(output / 'mic.wav'), 'wb') as wav:
                    wav.setparams((1, 2, 16000, 0, 'NONE', ''))
                    wav.writeframes(b''.join(struct.pack('<h', int(1200 * math.sin(n * .08)))
                                             for n in range(16000)))
                audio = cli('audio-input', device, str(output / 'mic.wav'), '--timeout', '15')
                assert audio['status'] == 'completed' and audio['consumed'] == 16000, audio
                cursor = uart.stat().st_size
                call('signals', changes={'charge_n': False})
                wait(b'batt charging st changed: not charging -> charging', cursor)
                call('signals', changes={'charge_n': True})
                cursor = uart.stat().st_size
                call('button_sequence', button='power', count=1, hold_ms=120, gap_ms=80)
                wait(rb'gpio_button\[0\]: short_press', cursor)
            elif boot == 1:
                shell('touch_trace replay', b'TOUCH_TRACE REPLAY START file=')
                wait(b'TOUCH_TRACE REPLAY PASS')
                shell('wifi connect SnapTest', rb'DHCP Success on VIF-0: IP=192\.0\.2\.2')
                wait(rb'route applied: default=wifi0 ip=192\.0\.2\.2')
                if args.http_host:
                    check_http(shell, args.http_host)
                if args.photo:
                    wait(rb'MCP message received: [^\r\n]*"initialize"')
                    with chart_frame(call, output / 'preview.png', review=False):
                        cursor = uart.stat().st_size
                        call('touch', x=100, y=95, pressed=True)
                        wait(rb'event\S*touch_down', cursor)
                        call('touch', x=100, y=95, pressed=False)
                        wait(b'preview crop: 320x240', cursor)
                        wait(b'DVP Normal capture started', cursor)
                    with chart_frame(call, output / 'photo.png', review=True):
                        call('button_sequence', button='ai', count=1, hold_ms=170, gap_ms=80)
                        wait(b'1280x960 slice_rows=8', cursor)
                        wait(rb'PTT seq=\d+ ret=0 [^\r\n]*strips=120', cursor)
                        wait(rb'captured jpeg [1-9]\d+ bytes soi=ffd8 eoi=ffd9 1280x960', cursor)
                shell('kv set int wifi_onoff 0', b'flash set wifi_onoff:0 success')
                shell('kv set int mobile_onoff 1', b'flash set mobile_onoff:1 success')
            else:
                # The ECM bridge is explicitly enabled for this run, independently of Wi-Fi.
                mcp = Client(binary, library)
                try:
                    target = {'id': device, 'run': run, 'modem': 'cellular'}
                    wait(b'NT26 USB network initialization succeeded')
                    modem = mcp.tool('lisem_modem', target)
                    assert not modem['network_enabled']
                    modem = mcp.tool('lisem_modem', {**target, 'network_enabled': True,
                                                     'usb_generation': modem['usb_generation']})
                    assert modem['network_enabled']
                finally:
                    mcp.close()
                wait(rb'4G ECM ready: modem=NT26 netif=ecm0 IPv4=192\.0\.2\.2')
                wait(rb'route applied: default=ecm0 ip=192\.0\.2\.2')
                if args.http_host:
                    check_http(shell, args.http_host)
                modem = cli('modem', device, 'cellular', '--run', run)
                assert modem['usb_configured'] and modem['pdp_active'] and modem['network_enabled'], modem
                assert modem['ecm_rx_frames'] > 0 and modem['ecm_tx_frames'] > 0, modem
            service.stop()
            assert (path / 'bl6138-flash.bin').read_bytes() == touch_image
        print('Snap-Learn: LPK, three cold boots, UID/OTP, TF touch trace, keys, ADC audio, Wi-Fi and ECM DHCP PASS')
        if args.http_host:
            print('Wi-Fi and ECM firmware HTTP requests to host PASS')
        if args.photo:
            print('Photo-question preview and 1280x960 JPEG with thumbnail PASS')
    finally:
        service.close()


if __name__ == '__main__':
    main()
