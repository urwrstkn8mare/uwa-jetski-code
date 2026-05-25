#!/usr/bin/env python3
"""Run idf.py build across selected project(s)."""

import argparse
import logging

from _project_cli import (
    add_common_logging_arg,
    add_project_arg,
    ensure_idf_env,
    resolve_project_paths,
    run_idf_subcommand,
    setup_logging,
)


def main() -> None:
    parser = argparse.ArgumentParser(
        "build.py",
        description=(
            "Run idf.py build in selected project(s). "
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
        operation_name="build",
    )

    errored = []
    for project_dir in projects:
        code = run_idf_subcommand(project_dir, "build", idf_args)
        if code != 0:
            errored.append(project_dir.name)

    if len(errored) > 0:
        logging.error(f"One or more builds failed: {', '.join(errored)}")
        raise SystemExit(1)
    logging.info("All builds completed successfully.")


if __name__ == "__main__":
    main()
