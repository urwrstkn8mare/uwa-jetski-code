#!/usr/bin/env python3
"""Set up clangd for selected project(s) by reconfiguring with the clang toolchain.

Runs idf.py reconfigure with IDF_TOOLCHAIN=clang into build.clang, then merges all
projects' compile_commands.json into the repo root so clangd can find every file.
No compilation is performed — this is purely for IDE language-server support.
"""

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
        "setup_clangd.py",
        description=(
            "Reconfigure project(s) with the clang toolchain (into build.clang) and "
            "merge compile_commands.json into the repo root for clangd. "
            "Without -p/--project: repo root targets all projects, project root targets itself. "
            "No compilation is performed."
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
        operation_name="setup_clangd",
    )

    pre_args = ["-B", "build.clang", "-D", "IDF_TOOLCHAIN=clang"]

    errored = []
    for project_dir in projects:
        code = run_idf_subcommand(project_dir, "reconfigure", idf_args, pre_args=pre_args)
        if code != 0:
            errored.append(project_dir.name)

    logging.info("Merging compile_commands.json from all clang build dirs...")
    merge_compile_commands(get_context().repo_root)

    if errored:
        logging.error(f"One or more reconfigures failed: {', '.join(errored)}")
        raise SystemExit(1)
    logging.info("clangd setup complete.")


if __name__ == "__main__":
    main()
