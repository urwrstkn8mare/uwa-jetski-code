#!/usr/bin/env python3
"""Run idf.py flash across selected project(s)."""

import argparse
import logging

from _project_cli import (
    add_common_logging_arg,
    add_project_arg,
    ensure_idf_env,
    load_serial_port,
    resolve_project_paths,
    run_idf_subcommand,
    setup_logging,
)


def main() -> None:
    parser = argparse.ArgumentParser(
        "flash.py",
        description=(
            "Run idf.py flash in selected project(s). "
            "Without -p/--project: repo root targets all projects, project root targets itself."
        ),
    )
    add_project_arg(parser)
    add_common_logging_arg(parser)
    args, idf_args = parser.parse_known_args()

    setup_logging(args.verbose)
    ensure_idf_env()

    projects = resolve_project_paths(
        raw_project_values=args.project,
        single_project_only=False,
        operation_name="flash",
    )

    has_error = False
    for project_dir in projects:
        serial_port = load_serial_port(project_dir)
        code = run_idf_subcommand(project_dir, "flash", ["-p", serial_port, *idf_args])
        has_error = has_error or code != 0

    if has_error:
        logging.error("One or more flash operations failed.")
        raise SystemExit(1)
    logging.info("All flash operations completed successfully.")


if __name__ == "__main__":
    main()
