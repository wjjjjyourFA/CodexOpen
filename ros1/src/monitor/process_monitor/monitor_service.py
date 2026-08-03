"""Main process-monitoring business workflow."""

from __future__ import annotations

import time

from common.program_models import MonitorConfig, ProgramStatus
from common.restart_handler import RestartHandler
from ui.terminal_ui import TerminalUI
from process_inspector import ProcessInspector


def collect_statuses(
    config: MonitorConfig,
    inspector: ProcessInspector,
    restarter: RestartHandler | None = None,
    auto_restart: bool = False,
) -> tuple[ProgramStatus, ...]:
    """Collect one reusable status snapshot for terminal or external adapters."""
    processes = inspector.scan()
    now = time.monotonic()
    statuses: list[ProgramStatus] = []
    for program in config.programs:
        matches = inspector.find_matches(program, processes)
        message = ""
        if auto_restart and restarter and len(matches) < program.min_instances:
            attempted, restart_message = restarter.restart(program, now)
            if attempted or restart_message != "单项未启用":
                message = restart_message
        statuses.append(ProgramStatus(program, matches, message))
    return tuple(statuses)


class MonitorService:
    def __init__(
        self,
        config: MonitorConfig,
        inspector: ProcessInspector,
        restarter: RestartHandler,
        ui: TerminalUI,
        interval: float,
        auto_restart: bool,
    ):
        self.config = config
        self.inspector = inspector
        self.restarter = restarter
        self.ui = ui
        self.interval = interval
        self.auto_restart = auto_restart
        self.stopping = False

    def stop(self) -> None:
        self.stopping = True

    def check_once(self) -> tuple[ProgramStatus, ...]:
        return collect_statuses(
            self.config,
            self.inspector,
            restarter=self.restarter,
            auto_restart=self.auto_restart,
        )

    def run(self, once: bool = False) -> None:
        while not self.stopping:
            statuses = self.check_once()
            content = self.ui.render(self.config, statuses, self.auto_restart)
            self.ui.display(content, clear=not once)
            if once:
                return
            deadline = time.monotonic() + self.interval
            while not self.stopping and time.monotonic() < deadline:
                time.sleep(min(0.2, max(0.0, deadline - time.monotonic())))
