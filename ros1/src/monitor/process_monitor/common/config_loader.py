"""Read and validate one shell-specific JSON configuration."""

from __future__ import annotations

import re
import json
from pathlib import Path
from typing import Any

from common.program_models import MonitorConfig, ProgramConfig, RestartConfig


class ConfigError(ValueError):
    pass


def _parse_restart(raw: dict[str, Any], program_name: str) -> RestartConfig:
    command = raw.get("command")
    if isinstance(command, list):
        command = tuple(str(part) for part in command)
    elif command is not None and not isinstance(command, str):
        raise ConfigError(f"{program_name}: restart.command 必须是字符串或字符串数组")

    cooldown = float(raw.get("cooldown_seconds", 30))
    if cooldown < 0:
        raise ConfigError(f"{program_name}: restart.cooldown_seconds 不能小于 0")
    return RestartConfig(
        enabled=bool(raw.get("enabled", False)),
        command=command,
        cwd=raw.get("cwd"),
        cooldown_seconds=cooldown,
    )


def _parse_program(raw: dict[str, Any]) -> ProgramConfig:
    try:
        name = str(raw["name"])
    except KeyError as exc:
        raise ConfigError("程序配置缺少 name") from exc

    match = raw.get("match", {})
    cmdline_regex = match.get("cmdline_regex")

    # 预编译正则表达式并在加载时校验语法
    cmdline_pattern = None
    if cmdline_regex:
        try:
            cmdline_pattern = re.compile(cmdline_regex)
        except re.error as exc:
            raise ConfigError(f"{name}: cmdline_regex 无效的正则表达式 ({exc})") from exc
        
    program = ProgramConfig(
        name=name,
        group=str(raw.get("group", "其他")),
        process_name=match.get("process_name"),
        cmdline_contains=tuple(str(token) for token in match.get("cmdline_contains", [])),
        cmdline_regex=cmdline_regex,
        cmdline_pattern=cmdline_pattern,
        min_instances=int(raw.get("min_instances", 1)),
        restart=_parse_restart(raw.get("restart", {}), name),
    )
    
    if program.min_instances < 1:
        raise ConfigError(f"{name}: min_instances 必须大于等于 1")
    if not (program.process_name or program.cmdline_contains or program.cmdline_regex):
        raise ConfigError(f"{name}: 至少需要一种 match 匹配规则")
    return program


def load_config(path: Path) -> MonitorConfig:
    try:
        with path.open("r", encoding="utf-8") as stream:
            raw = json.load(stream)
    except (OSError, json.JSONDecodeError) as exc:
        raise ConfigError(f"无法读取 {path}: {exc}") from exc

    try:
        profile = str(raw["profile"])
        refresh_seconds = float(raw.get("refresh_seconds", 2))
        raw_programs = raw["programs"]
    except (KeyError, TypeError, ValueError) as exc:
        raise ConfigError(f"{path}: 顶层配置格式错误") from exc
    if refresh_seconds <= 0:
        raise ConfigError("refresh_seconds 必须大于 0")
    if not isinstance(raw_programs, list):
        raise ConfigError("programs 必须是数组")

    programs = tuple(
        _parse_program(item) for item in raw_programs if item.get("enabled", True)
    )
    names = [program.name for program in programs]
    if len(names) != len(set(names)):
        raise ConfigError(f"{path}: 程序 name 不能重复")
    return MonitorConfig(profile, refresh_seconds, programs)


def profile_config_path(config_dir: Path, profile: str) -> Path:
    """Resolve a profile without allowing it to escape the config directory."""
    if not profile or Path(profile).name != profile:
        raise ConfigError(f"非法 profile 名称: {profile!r}")
    return config_dir / f"{profile}.json"
