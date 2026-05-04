#!/usr/bin/env python3
"""Run idf.py monitor with project-selected serial port."""

import argparse
import logging
import subprocess
import sys

from _project_cli import (
    add_common_logging_arg,
    add_project_arg,
    ensure_idf_env,
    load_serial_port,
    resolve_project_paths,
    setup_logging,
)

def main():
    terminal_is_interactive = sys.stdin.isatty()
    parser = argparse.ArgumentParser(
        "monitor.py",
        description=(
            "Run idf.py monitor with serial port from .serial_port. "
            "Without -p/--project: project root targets itself."
        ),
    )
    add_project_arg(parser)
    add_common_logging_arg(parser)
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
    setup_logging(args.verbose)

    ensure_idf_env()
    project_dir = resolve_project_paths(
        raw_project_values=args.project,
        single_project_only=True,
        operation_name="monitor",
    )[0]

    port = load_serial_port(project_dir)

    logging.info(
        "[%s] Starting idf.py monitor on %s with timeout=%ss",
        project_dir.name,
        port,
        args.timeout,
    )
    monitor_cmd = ["idf.py", "monitor", "-p", port, *args.rest]
    proc = subprocess.Popen(monitor_cmd, cwd=project_dir)

    def stop_monitor(grace_seconds: float = 0.8) -> int:
        """Stop monitor process, escalating only if needed."""
        if proc.poll() is not None:
            logging.info("EXIT_PATH: already_exited code=%s", proc.returncode)
            return proc.returncode
        logging.info("EXIT_PATH: timeout_sigterm")
        proc.terminate()
        try:
            code = proc.wait(timeout=grace_seconds)
            logging.info("EXIT_PATH: sigterm_clean code=%s", code)
            return code
        except subprocess.TimeoutExpired:
            logging.warning("EXIT_PATH: sigterm_failed_sigkill")
            proc.kill()
            code = proc.wait()
            logging.info("EXIT_PATH: sigkill code=%s", code)
            return code

    if args.timeout > 0:
        try:
            exit_code = proc.wait(timeout=args.timeout)
            logging.info("EXIT_PATH: timeout_window_normal_exit code=%s", exit_code)
        except subprocess.TimeoutExpired:
            logging.info("Timeout reached (%ss). Stopping monitor.", args.timeout)
            exit_code = stop_monitor()
    else:
        exit_code = proc.wait()
        logging.info("EXIT_PATH: normal_exit code=%s", exit_code)

    logging.info("Monitor session finished.")
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
