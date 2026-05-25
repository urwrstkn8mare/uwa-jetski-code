#!/usr/bin/env python3
"""Run idf.py build across selected project(s)."""

import argparse
import logging

from _project_cli import (
    add_common_logging_arg,
    add_project_arg,
    ensure_idf_env,
    get_context,
    resolve_project_paths,
    run_idf_subcommand,
    setup_logging,
)
from merge_compile_commands import merge as merge_compile_commands


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
    parser.add_argument(
        "-c", "--clang", action="store_true", help="Build with clang in build.clang"
    )
    args, idf_args = parser.parse_known_args()

    setup_logging(args.verbose)
    ensure_idf_env()

    projects = resolve_project_paths(
        raw_project_values=args.project,
        single_project_only=False,
        operation_name="build",
    )

    pre_args = []
    if args.clang:
        pre_args = ["-B", "build.clang", "-D", "IDF_TOOLCHAIN=clang"]

    errored = []
    for project_dir in projects:
        code = run_idf_subcommand(project_dir, "build", idf_args, pre_args=pre_args)
        if code != 0:
            errored.append(project_dir.name)

    if args.clang:
        logging.info("Merging compile_commands.json from all clang build dirs...")
        merge_compile_commands(get_context().repo_root)

    if len(errored) > 0:
        logging.error(f"One or more builds failed: {', '.join(errored)}")
        raise SystemExit(1)
    logging.info("All builds completed successfully.")


if __name__ == "__main__":
    main()
