"""
Stream a USB camera to the ground station: capture with OpenCV and send JPEG frames over TCP.

Run it on the computer the camera is plugged into, after ``pip install opencv-python``::

    python -m waybionic_camera.camera_bridge

Each frame carries the time it was captured, so the ground station can measure the delay.
This file needs only Python 3 and OpenCV, not ROS.
"""

import argparse
import socket
import sys
import time

from waybionic_camera.frames import pack


def open_camera(cv2, args):
    source = int(args.camera) if args.camera.isdigit() else args.camera
    backends = {'any': cv2.CAP_ANY, 'dshow': cv2.CAP_DSHOW, 'msmf': cv2.CAP_MSMF,
                'v4l2': cv2.CAP_V4L2, 'avfoundation': cv2.CAP_AVFOUNDATION}
    capture = cv2.VideoCapture(source, backends[args.backend])
    if not capture.isOpened():
        raise SystemExit(f'Could not open camera {args.camera}')
    # USB 2 webcams only reach 1080p at 30 fps when they send MJPG.
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    capture.set(cv2.CAP_PROP_FPS, args.fps)
    return capture


def main():
    parser = argparse.ArgumentParser(description='Send a camera to the ground station.')
    parser.add_argument('--host', default='127.0.0.1', help='ground station address')
    parser.add_argument('--port', type=int, default=47310, help='camera_receiver port')
    parser.add_argument('--camera', default='0', help='camera index, video file or stream URL')
    parser.add_argument('--backend', default='any',
                        choices=('any', 'dshow', 'msmf', 'v4l2', 'avfoundation'))
    parser.add_argument('--width', type=int, default=1920)
    parser.add_argument('--height', type=int, default=1080)
    parser.add_argument('--fps', type=float, default=30.0)
    parser.add_argument('--quality', type=int, default=80, help='JPEG quality, 1-100')
    args = parser.parse_args()
    try:
        import cv2
    except ImportError:
        raise SystemExit('The camera bridge needs OpenCV: pip install opencv-python') from None
    capture = open_camera(cv2, args)
    number, sender, report = 0, None, time.monotonic()
    next_frame = report
    print(f'Sending camera {args.camera} to {args.host}:{args.port} (Ctrl+C to stop)')
    try:
        while True:
            ok, image = capture.read()
            stamp_ns = time.time_ns()
            if not ok:
                if not args.camera.isdigit():
                    # Loop a video file so it can stand in for a camera.
                    capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                raise SystemExit('The camera stopped sending frames')
            if sender is None:
                try:
                    sender = socket.create_connection((args.host, args.port), timeout=1.0)
                    sender.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    print('Connected to the ground station')
                except OSError:
                    time.sleep(0.5)
                    continue
            ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, args.quality])
            height, width = image.shape[:2]
            number += 1
            try:
                sender.sendall(pack(number, stamp_ns, width, height, jpeg.tobytes()))
            except OSError:
                print('Ground station disconnected; retrying')
                sender.close()
                sender = None
            if number % 30 == 0:
                now = time.monotonic()
                print(f'\r{30 / (now - report):5.1f} fps at {width}x{height}', end='',
                      flush=True)
                report = now
            if not args.camera.isdigit() and args.fps > 0:
                # A camera sets its own pace; a video file is played at the requested rate.
                next_frame += 1.0 / args.fps
                time.sleep(max(0.0, next_frame - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        capture.release()
        if sender is not None:
            sender.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
