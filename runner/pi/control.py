#!/usr/bin/env python3
"""Send one command to the running Pi companion, without opening its flight UART.

  python3 runner/pi/control.py raspberrypi.local calibrate
  python3 runner/pi/control.py raspberrypi.local autotune status
No third-party packages. Commands are sent once; a timeout is not a rejection.
"""
import argparse
import socket
import sys


def request(host, port, command, timeout=2.0):
    if not command or "\n" in command or "\r" in command or "\0" in command:
        raise ValueError("send one nonempty command at a time")
    payload = (command + "\n").encode("utf-8")
    if len(payload) > 500:
        raise ValueError("command is too long")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(timeout)
        sock.connect((host, port))
        sock.send(payload)
        return sock.recv(4096).decode("utf-8", errors="replace").strip()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host", help="Pi hostname/IP, or 127.0.0.1 over SSH")
    parser.add_argument("command", nargs="+", help="e.g. learning, keep on, autotune attitude")
    parser.add_argument("--port", type=int, default=14560)
    parser.add_argument("--timeout", type=float, default=2.0)
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535 or args.timeout <= 0:
        parser.error("port must be 1..65535 and timeout must be positive")
    try:
        print(request(args.host, args.port, " ".join(args.command), args.timeout))
        return 0
    except (OSError, ValueError) as error:
        print(f"No acknowledgement: {error}. The command may have arrived; query status before retrying.", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
