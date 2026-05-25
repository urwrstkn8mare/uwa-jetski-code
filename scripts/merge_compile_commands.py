#!/usr/bin/env python3
"""Merge compile_commands.json from all project build.clang dirs into the repo root."""

import json
import logging

from _project_cli import add_common_logging_arg, get_context, setup_logging

import argparse
from pathlib import Path


def merge(repo_root: Path) -> None:
    logger = logging.getLogger("merge_compile_commands")
    projects_dir = repo_root / "projects"

    entries: dict[str, dict] = {}
    for db_path in sorted(projects_dir.glob("*/build.clang/compile_commands.json")):
        project_name = db_path.parts[-3]
        try:
            loaded = json.loads(db_path.read_text())
        except Exception as e:
            logger.warning("[%s] Could not read %s: %s", project_name, db_path, e)
            continue
        before = len(entries)
        for entry in loaded:
            entries[entry["file"]] = entry
        logger.info("[%s] Added %d entries (%d new)", project_name, len(loaded), len(entries) - before)

    if not entries:
        raise SystemExit("Error: no compile_commands.json files found under projects/*/build.clang/")

    out_path = repo_root / "compile_commands.json"
    out_path.write_text(json.dumps(list(entries.values()), indent=2))
    logger.info("Wrote %d total entries to %s", len(entries), out_path)


def main() -> None:
    parser = argparse.ArgumentParser(
        "merge_compile_commands.py",
        description="Merge compile_commands.json from all project build.clang dirs into repo root.",
    )
    add_common_logging_arg(parser)
    args = parser.parse_args()
    setup_logging(args.verbose)
    merge(get_context().repo_root)


if __name__ == "__main__":
    main()
