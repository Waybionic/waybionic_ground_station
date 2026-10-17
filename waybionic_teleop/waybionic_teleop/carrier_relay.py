"""
Relay the WayBionic carrier's COM port to a local TCP port for the drive node in Docker.

Docker Desktop cannot open Windows COM ports, but the drive node opens the carrier with
pyserial's serial_for_url, so it accepts socket://host.docker.internal:PORT as its
drive_channel. This passes bytes both ways unchanged, as soon as they arrive. If the carrier
is unplugged or the bridge stops, the connection closes, so the drive node loses the carrier
exactly as it does when the USB cable is pulled on a Mac or Linux. Needs pyserial. Run from
the waybionic_teleop directory:

    python -m waybionic_teleop.carrier_relay --port COM5
"""

import argparse
import socket
import threading

import serial
from serial.tools import list_ports

ARDUINO_VID = 0x2341
DEFAULT_BAUDRATE = 1000000
DEFAULT_TCP_PORT = 47320
REOPEN_EVERY_S = 1.0
# How often blocking calls wake up to check whether to stop.
POLL_S = 0.05
CHUNK = 4096


def find_carrier():
    """Return the only Arduino COM port, or None if there are none or several."""
    ports = [info.device for info in list_ports.comports() if info.vid == ARDUINO_VID]
    return ports[0] if len(ports) == 1 else None


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
                server = socket.create_server((bind, tcp_port))
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
                        help='carrier COM port, such as COM5 (default: the only Arduino)')
    parser.add_argument('--baudrate', type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument('--bind', default='127.0.0.1',
                        help='listen address; keep it local, this drives the arm')
    parser.add_argument('--tcp-port', type=int, default=DEFAULT_TCP_PORT)
    args = parser.parse_args()
    port = args.port or find_carrier()
    if port is None:
        found = ', '.join(info.device for info in list_ports.comports()) or 'none'
        raise SystemExit(f'Plug in only the carrier, or pass --port. COM ports found: {found}')
    print(f'Relaying {port} to {args.bind}:{args.tcp_port} (Ctrl+C to stop)')
    print(f'Ground station: drive_channel:=socket://host.docker.internal:{args.tcp_port}')
    stop = threading.Event()
    try:
        serve(lambda: serial.Serial(port, args.baudrate, timeout=POLL_S),
              args.bind, args.tcp_port, stop)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()


if __name__ == '__main__':
    main()
