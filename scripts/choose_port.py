#!/usr/bin/env python3
"""Choose from available ports (for ESP-IDF)"""

import argparse
from pathlib import Path

import serial.tools.list_ports as list_ports
import sys


def main():
    parser = argparse.ArgumentParser(
        "choose_port.py",
        description="Choose from available ports (for ESP-IDF). Will save chosen port to .serial_port file so that port can be used whenever running a script from project root. Running without a TTY will just print available ports.",
    )

    available_ports = [p for p in list_ports.comports() if p.description != "n/a"]
    parser.add_argument(
        "-p",
        "--port",
        type=str,
        default="",
        choices=[p.device for p in available_ports],
    )
    parser.add_argument("-i", "--interactive", type=bool, default=sys.stdin.isatty())
    args = parser.parse_args()

    def choose_port(device):
        path = Path(".serial_port")
        path.write_text(device)
        print(f"Written '{device}' to {path}")

    if args.port != "":
        choose_port(args.port)
    else:
        if args.interactive:
            print("Choose from: ")
        for i, port in enumerate(available_ports):
            if args.interactive:
                print(f"{i + 1}: ", end="")
            print(port.device)
        if args.interactive:
            choice = available_ports[int(input("Choice: ")) - 1]
            choose_port(choice.device)

    print("\n--- Done ---")


if __name__ == "__main__":
    main()
