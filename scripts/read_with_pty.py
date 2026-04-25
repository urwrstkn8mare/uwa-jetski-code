#!/usr/bin/env python3
"""Create a PTY with idf.py monitor and capture output"""

import os
import pty
import select
import subprocess
import time
import argparse
import serial.tools.list_ports as list_ports


def main():
    parser = argparse.ArgumentParser(
        "read_with_pty.py",
        description="Create a PTY with idf.py monitor and print output. Runs for <timeout> seconds on <port> port. Must run in the project directory with the ESP-IDF environment activated.",
    )
    parser.add_argument("-t", "--timeout", type=int, default=20)
    parser.add_argument(
        "-p",
        "--port",
        type=str,
        required=True,
        choices=[p.device for p in list_ports.comports() if p.description != "n/a"],
    )
    args = parser.parse_args()

    if os.environ.get("IDF_PATH") is None:
        raise Exception("Not running is an ESP-IDF environment!")

    master, slave = pty.openpty()
    pty_name = os.ttyname(slave)
    print(f"Created PTY: {pty_name}")

    proc = subprocess.Popen(
        ["idf.py", "-p", args.port, "monitor"],
        stdin=slave,
        stdout=slave,
        stderr=slave,
        close_fds=True,
    )

    print(f"Reading output from {args.port} ({args.timeout} timeout)...")
    start = time.time()
    while time.time() - start < args.timeout and proc.poll() is None:
        r, _, _ = select.select([master], [], [], 0.5)
        if r:
            data = os.read(master, 1024)
            if data:
                text = data.decode("utf-8", errors="replace")
                print(text, end="")

    os.close(master)
    if proc.poll() is None:
        proc.terminate()
    print("\n--- Done ---")


if __name__ == "__main__":
    main()
