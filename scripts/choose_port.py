#!/usr/bin/env python3
"""Choose from available ports (for ESP-IDF)"""

import argparse
import logging

import serial.tools.list_ports as list_ports
import sys

from _project_cli import (
    add_common_logging_arg,
    add_project_arg,
    resolve_project_paths,
    setup_logging,
)


def main():
    parser = argparse.ArgumentParser(
        "choose_port.py",
        description=(
            "Choose from available ports (for ESP-IDF). "
            "Saves chosen port to a project's .serial_port file."
        ),
    )
    add_project_arg(parser)
    add_common_logging_arg(parser)

    available_ports = [p for p in list_ports.comports() if p.description != "n/a"]
    parser.add_argument(
        "-P",
        "--port",
        type=str,
        default="",
        choices=[p.device for p in available_ports],
    )
    parser.add_argument("-i", "--interactive", type=bool, default=sys.stdin.isatty())
    args = parser.parse_args()
    setup_logging(args.verbose)

    project_dir = resolve_project_paths(
        raw_project_values=args.project,
        single_project_only=True,
        operation_name="choose_port",
    )[0]

    def choose_port(device: str):
        path = project_dir / ".serial_port"
        path.write_text(device)
        logging.info("[%s] Saved serial port '%s' to %s", project_dir.name, device, path)

    if args.port != "":
        choose_port(args.port)
    else:
        if not available_ports:
            raise SystemExit("Error: no serial ports detected.")
        if args.interactive:
            logging.info("Choose from:")
        for i, port in enumerate(available_ports):
            if args.interactive:
                print(f"{i + 1}: {port.device}")
            else:
                print(port.device)
        if args.interactive:
            choice = available_ports[int(input("Choice: ")) - 1]
            choose_port(choice.device)

    logging.info("Port selection operation complete.")


if __name__ == "__main__":
    main()
