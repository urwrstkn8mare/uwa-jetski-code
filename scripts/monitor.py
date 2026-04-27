#!/usr/bin/env python3
"""Script that either activates idf.py monitor with the port defined in .serial_port automatically AND activates a PTY automatically as well"""

import os
import pty
import tty
import termios
import time
import argparse
from pathlib import Path

import sys
import select

CTRL_BRACKET_R = b"\x1d"


def main():
    terminal_is_interactive = sys.stdin.isatty()
    parser = argparse.ArgumentParser(
        "read_with_pty.py",
        description="Create a PTY with idf.py monitor and print output. Runs for <timeout> seconds on port defined in .serial_port. Must run in the project directory with the ESP-IDF environment activated. Can also be used interactively like normal.",
    )
    parser.add_argument(
        "-t",
        "--timeout",
        type=int,
        default=20 if not terminal_is_interactive else -1,
        help="-1 value implies no timeout",
    )
    parser.add_argument(
        "rest", nargs=argparse.REMAINDER, help="Gets passed to idf monitor command"
    )
    args = parser.parse_args()

    if os.environ.get("IDF_PATH") is None:
        raise Exception("Not running is an ESP-IDF environment!")

    port = Path(".serial_port").read_text().strip()

    pid, master_fd = pty.fork()

    if pid == 0:  # child proc
        print(f"--- STARTING idf.py monitor on {port} for {args.timeout}s ---")
        os.execvp("idf.py", ["idf.py", "monitor", "-p", port, *args.rest])
    else:  # parent proc
        old_tty = None
        try:
            if terminal_is_interactive:
                old_tty = termios.tcgetattr(sys.stdin.fileno())
                tty.setraw(sys.stdin.fileno())

            trigger_time = time.time() + args.timeout
            sent_flag = False
            stdin_fd = sys.stdin.fileno() if terminal_is_interactive else None
            while True:
                # Calculate how much time is left until the injection
                current_time = time.time()
                if sent_flag or args.timeout < 0:
                    remaining = None
                else:
                    remaining = max(0, trigger_time - current_time)

                # select() will block until data is ready OR 'remaining' seconds pass
                select_fds = [master_fd]
                if stdin_fd is not None:
                    select_fds.append(stdin_fd)
                rfds, _, _ = select.select(select_fds, [], [], remaining)

                # Check if the timer hit while we were blocking
                if not sent_flag and args.timeout > 0 and time.time() >= trigger_time:
                    print(
                        f"--- SENDING STOP SIGNAL (CTRL-]) as {args.timeout}s timeout passed ---"
                    )
                    os.write(master_fd, CTRL_BRACKET_R)
                    sent_flag = True

                # Forward child output to your terminal
                if master_fd in rfds:
                    try:
                        data = os.read(master_fd, 1024)
                        if not data:
                            break
                        os.write(sys.stdout.fileno(), data)
                    except OSError as e:
                        if e.errno == 5:  # EIO when slave side closes
                            break
                        raise

                # Forward your typing to the child
                if stdin_fd is not None and stdin_fd in rfds:
                    data = os.read(stdin_fd, 1024)
                    if not data:
                        break
                    os.write(master_fd, data)

        finally:
            if old_tty is not None:
                termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_tty)
            os.close(master_fd)
            os.waitpid(pid, 0)

    print("\n--- Done ---")


if __name__ == "__main__":
    main()
