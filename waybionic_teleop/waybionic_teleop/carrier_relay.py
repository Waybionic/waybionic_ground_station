"""
Relay the WayBionic carrier's COM port to a local TCP port for the drive node in Docker.

Docker Desktop cannot open Windows COM ports, but the drive node opens the carrier with
pyserial's serial_for_url, so it accepts socket://host.docker.internal:PORT as its
drive_channel. This passes bytes both ways unchanged, as soon as they arrive. If the carrier
is unplugged or the bridge stops, the connection closes, so the drive node loses the carrier
exactly as it does when the USB cable is pulled on a Mac or Linux. Needs pyserial. Run from
the waybionic_teleop directory:

    python -m waybionic_teleop.carrier_relay

It looks for the Arduino's COM port each time it opens the carrier, so a carrier plugged in
after it starts, or replugged under another COM number, is found. A board whose USB-serial chip
is not Arduino's is used if it is the only USB serial port. With several candidates, name the
carrier's: --port COM5.
"""

import argparse
import socket
import threading
import traceback

import serial
from serial.tools import list_ports

ARDUINO_VID = 0x2341
DEFAULT_BAUDRATE = 1000000
DEFAULT_TCP_PORT = 47320
REOPEN_EVERY_S = 1.0
# How often blocking calls wake up to check whether to stop.
POLL_S = 0.05
CHUNK = 4096


def describe(info):
    """Return a COM port's name with what Windows calls it, as the operator sees it listed."""
    description = getattr(info, 'description', '')
    if description and description != 'n/a':
        return f'{info.device} ({description})'
    return info.device


def make_log(path):
    """Return log(), which prints each message and, given a path, appends it to that file too."""
    def log(message):
        print(message, flush=True)
        if path:
            try:
                with open(path, 'a', encoding='utf-8') as stream:
                    stream.write(f'{message}\n')
            except OSError:
                pass
    return log


def carrier_opener(port, baudrate, open_serial=serial.Serial, list_com_ports=list_ports.comports,
                   log=print):
    """
    Return the open_port that serve() calls whenever it needs the carrier.

    With no port it looks for the carrier every time, because the carrier may be plugged in
    after the relay starts, still be enumerating, or come back under another COM number. It
    takes the only Arduino, or failing that the only USB serial port, as a clone board's
    USB-serial chip is not Arduino's. It never guesses between several.
    """
    def open_carrier():
        name = port
        if name is None:
            infos = list_com_ports()
            arduinos = sorted(info.device for info in infos if info.vid == ARDUINO_VID)
            usb = sorted((info for info in infos if info.vid is not None),
                         key=lambda info: info.device)
            if len(arduinos) > 1:
                raise serial.SerialException(
                    f'several Arduino COM ports ({", ".join(arduinos)}); plug in only the '
                    'carrier, or pass --port')
            if arduinos:
                name = arduinos[0]
            elif len(usb) == 1:
                name = usb[0].device
                log(f'{describe(usb[0])} does not show the Arduino USB vendor ID, but it is the '
                    'only USB serial port, so it is used as the carrier. Pass --port if that '
                    'is wrong.')
            elif usb:
                raise serial.SerialException(
                    'no Arduino COM port found, and several USB serial ports '
                    f'({", ".join(describe(info) for info in usb)}); plug in only the carrier, '
                    'or pass --port')
            else:
                listed = ', '.join(describe(info) for info in sorted(
                    infos, key=lambda info: info.device)) or 'none'
                raise serial.SerialException(
                    f'no Arduino COM port found (COM ports: {listed}); plug in the carrier')
        link = open_serial(name, baudrate, timeout=POLL_S)
        log(f'Carrier on {name}')
        return link
    return open_carrier


def relay(link, client, stop):
    """
    Pass bytes both ways between the carrier and one TCP client until either side fails.

    Return 'client' if the client disconnected, 'serial' if the carrier was lost, or
    'stopped' once stop is set. Always closes the client; never closes the carrier.
    """
    try:
        # Send each frame at once instead of waiting to fill a packet.
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except OSError:
        pass  # Not a TCP socket, as in the tests.
    client.settimeout(POLL_S)
    done = threading.Event()
    lock = threading.Lock()
    reasons = []

    def finish(reason):
        with lock:
            if not done.is_set():
                reasons.append(reason)
                done.set()

    def to_client():
        while not done.is_set():
            try:
                data = link.read(link.in_waiting or 1)
            except (serial.SerialException, OSError):
                finish('serial')
                return
            if not data:
                continue
            try:
                client.sendall(data)
            except OSError:
                finish('client')
                return

    def to_carrier():
        while not done.is_set():
            try:
                data = client.recv(CHUNK)
            except socket.timeout:
                continue
            except OSError:
                finish('client')
                return
            if not data:
                finish('client')
                return
            try:
                link.write(data)
            except (serial.SerialException, OSError):
                finish('serial')
                return

    threads = [threading.Thread(target=pump, daemon=True) for pump in (to_client, to_carrier)]
    for thread in threads:
        thread.start()
    try:
        while not done.wait(POLL_S):
            if stop.is_set():
                finish('stopped')
    finally:
        done.set()
        for thread in threads:
            thread.join()
        client.close()
    return reasons[0] if reasons else 'stopped'


def _carrier_alive(link):
    """Discard what the carrier sent with nobody connected; return False if it is gone."""
    try:
        link.reset_input_buffer()
    except (serial.SerialException, OSError):
        return False
    return True


def _close(thing):
    if thing is not None:
        try:
            thing.close()
        except (serial.SerialException, OSError):
            pass
    return None


def serve(open_port, bind, tcp_port, stop, log=print):
    """
    Relay the carrier to one client at a time until stop is set.

    Listen only while the carrier is open, so the drive node's connection is refused, as an
    unplugged port would be, until the carrier is back.
    """
    link = server = None
    problem = None
    try:
        while not stop.is_set():
            if link is None:
                try:
                    link = open_port()
                except (serial.SerialException, OSError) as error:
                    if str(error) != problem:
                        problem = str(error)
                        log(f'Carrier unavailable, retrying every second: {error}')
                    stop.wait(REOPEN_EVERY_S)
                    continue
                problem = None
                try:
                    server = socket.create_server((bind, tcp_port))
                except OSError as error:
                    raise SystemExit(
                        f'Port {tcp_port} is already in use ({error}).'
                        'Another carrier bridge may still be running, close it first.') from None
                server.settimeout(POLL_S)
                log('Carrier open; waiting for the ground station')
            try:
                client, address = server.accept()
            except socket.timeout:
                reason = None if _carrier_alive(link) else 'serial'
            else:
                log(f'Ground station connected from {address[0]}')
                reason = relay(link, client, stop)
            if reason == 'serial':
                log('Carrier lost; connection closed')
                server = _close(server)
                link = _close(link)
            elif reason == 'client':
                log('Ground station disconnected; waiting for it to reconnect')
    finally:
        _close(server)
        _close(link)


def main():
    parser = argparse.ArgumentParser(description='Relay the carrier COM port to the arm.')
    parser.add_argument('--port',
                        help='carrier COM port, such as COM5 (default: the only Arduino, '
                             'looked for again whenever the carrier is opened)')
    parser.add_argument('--baudrate', type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument('--bind', default='127.0.0.1',
                        help='listen address; keep it local, this drives the arm')
    parser.add_argument('--tcp-port', type=int, default=DEFAULT_TCP_PORT)
    parser.add_argument('--log-file',
                        help='also write what it says to this file, which the launcher shows '
                             'if the bridge does not start')
    args = parser.parse_args()
    log = make_log(args.log_file)
    log(f'Relaying the carrier to {args.bind}:{args.tcp_port} (Ctrl+C to stop)')
    log(f'Ground station: drive_channel:=socket://host.docker.internal:{args.tcp_port}')
    stop = threading.Event()
    try:
        serve(carrier_opener(args.port, args.baudrate, log=log), args.bind, args.tcp_port, stop,
              log)
    except KeyboardInterrupt:
        pass
    except SystemExit as error:
        log(str(error.code))
        raise
    except Exception:
        log(traceback.format_exc())
        raise
    finally:
        stop.set()


if __name__ == '__main__':
    main()
