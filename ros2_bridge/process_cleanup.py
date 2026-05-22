"""Process cleanup helpers for simulator wrapper processes."""

from __future__ import annotations

import os
import signal
import subprocess
import time
from collections import defaultdict
from typing import Iterable


ProcessRow = tuple[int, int, str]


def collect_matching_descendants(
    processes: Iterable[ProcessRow],
    *,
    current_pid: int,
    command_substring: str,
) -> list[int]:
    """Return descendants of current_pid whose command contains command_substring."""

    children_by_parent: dict[int, list[tuple[int, str]]] = defaultdict(list)
    for pid, ppid, command in processes:
        children_by_parent[ppid].append((pid, command))

    matching_pids: list[int] = []
    stack = list(children_by_parent.get(current_pid, ()))
    while stack:
        pid, command = stack.pop()
        if command_substring in command:
            matching_pids.append(pid)
        stack.extend(children_by_parent.get(pid, ()))

    return sorted(matching_pids)


def cleanup_matching_descendants(
    *,
    command_substring: str,
    current_pid: int | None = None,
    wait_s: float = 2.0,
) -> None:
    """Terminate matching child processes without killing parent wrappers."""

    current_pid = current_pid or os.getpid()
    pids = collect_matching_descendants(
        _read_process_table(),
        current_pid=current_pid,
        command_substring=command_substring,
    )

    for pid in pids:
        _signal_process(pid, signal.SIGTERM)

    if pids:
        time.sleep(wait_s)

    remaining = set(
        collect_matching_descendants(
            _read_process_table(),
            current_pid=current_pid,
            command_substring=command_substring,
        )
    )
    for pid in remaining:
        _signal_process(pid, signal.SIGKILL)


def _read_process_table() -> list[ProcessRow]:
    result = subprocess.run(
        ["ps", "-eo", "pid=,ppid=,command="],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        return []

    rows: list[ProcessRow] = []
    for line in result.stdout.splitlines():
        parts = line.strip().split(maxsplit=2)
        if len(parts) < 3:
            continue
        try:
            rows.append((int(parts[0]), int(parts[1]), parts[2]))
        except ValueError:
            continue
    return rows


def _signal_process(pid: int, sig: signal.Signals) -> None:
    try:
        os.kill(pid, sig)
    except ProcessLookupError:
        return
    except Exception as exc:
        print(f"Failed to signal process {pid}: {exc}")
