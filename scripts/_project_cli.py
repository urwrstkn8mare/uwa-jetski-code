#!/usr/bin/env python3
"""Shared project selection and command execution helpers for scripts."""

from __future__ import annotations

import argparse
import logging
import os
import shlex
import subprocess
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class ScriptContext:
    repo_root: Path
    projects_dir: Path
    cwd: Path


def get_context() -> ScriptContext:
    repo_root = Path(__file__).resolve().parent.parent
    projects_dir = repo_root / "projects"
    return ScriptContext(repo_root=repo_root, projects_dir=projects_dir, cwd=Path.cwd())


def setup_logging(verbose: bool = False) -> None:
    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.INFO,
        format="[%(levelname)s] %(message)s",
    )


def _list_project_names(projects_dir: Path) -> list[str]:
    if not projects_dir.exists():
        return []
    return sorted(
        p.name for p in projects_dir.iterdir() if p.is_dir() and (p / "CMakeLists.txt").exists()
    )


def _parse_project_flags(raw_values: list[str] | None) -> list[str]:
    if not raw_values:
        return []

    names: list[str] = []
    seen: set[str] = set()
    for value in raw_values:
        for name in (part.strip() for part in value.split(",")):
            if not name or name in seen:
                continue
            names.append(name)
            seen.add(name)
    return names


def add_project_arg(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "-p",
        "--project",
        action="append",
        help=(
            "Target project name. Repeat flag or use comma-separated values for multiple "
            "projects."
        ),
    )


def add_common_logging_arg(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Enable verbose logging.",
    )


def resolve_project_paths(
    *,
    raw_project_values: list[str] | None,
    single_project_only: bool,
    operation_name: str,
) -> list[Path]:
    logger = logging.getLogger("project_cli")
    ctx = get_context()
    project_names = _list_project_names(ctx.projects_dir)
    if not project_names:
        raise SystemExit("Error: no projects found under ./projects")
    logger.debug("Discovered projects: %s", ", ".join(project_names))

    selected_names = _parse_project_flags(raw_project_values)
    if selected_names:
        logger.info("Resolving projects from -p/--project: %s", ", ".join(selected_names))
        invalid = [name for name in selected_names if name not in project_names]
        if invalid:
            raise SystemExit(
                f"Error: unknown project(s): {', '.join(invalid)}. "
                f"Valid projects: {', '.join(project_names)}"
            )
    else:
        if ctx.cwd == ctx.repo_root:
            if single_project_only:
                raise SystemExit(
                    f"Error: {operation_name} supports only one project at a time. "
                    "Run from a project root or pass -p/--project."
                )
            selected_names = project_names
            logger.info("No project specified at repo root. Using all projects.")
        elif ctx.cwd.parent == ctx.projects_dir and ctx.cwd.name in project_names:
            selected_names = [ctx.cwd.name]
            logger.info("No project specified in project root. Using '%s'.", ctx.cwd.name)
        else:
            raise SystemExit(
                "Error: run from repo root, from a project root under ./projects, "
                "or pass -p/--project."
            )

    if single_project_only and len(selected_names) != 1:
        raise SystemExit(f"Error: {operation_name} accepts exactly one project.")

    paths = [ctx.projects_dir / name for name in selected_names]
    logger.debug("Resolved project paths: %s", ", ".join(str(p) for p in paths))
    return paths


def ensure_idf_env() -> None:
    logger = logging.getLogger("project_cli")
    if "IDF_PATH" not in os.environ:
        raise SystemExit(
            "Error: ESP-IDF environment not loaded. Run '. ./activate_scripts.sh' first."
        )
    logger.debug("ESP-IDF environment detected.")


def run_idf_subcommand(project_dir: Path, subcommand: str, extra_args: list[str]) -> int:
    logger = logging.getLogger("project_cli")
    command = ["idf.py", subcommand, *extra_args]
    logger.info(
        "[%s] Running: %s",
        project_dir.name,
        " ".join(shlex.quote(arg) for arg in command),
    )
    completed = subprocess.run(command, cwd=project_dir)
    logger.info("[%s] Exit code: %s", project_dir.name, completed.returncode)
    return completed.returncode


def load_serial_port(project_dir: Path) -> str:
    logger = logging.getLogger("project_cli")
    path = project_dir / ".serial_port"
    if not path.exists():
        raise SystemExit(
            f"Error: missing {path}. Run choose_port.py for project '{project_dir.name}' first."
        )
    value = path.read_text(encoding="utf-8").strip()
    if not value:
        raise SystemExit(
            f"Error: {path} is empty. Run choose_port.py for project '{project_dir.name}'."
        )
    logger.debug("[%s] Loaded serial port '%s' from %s", project_dir.name, value, path)
    return value
