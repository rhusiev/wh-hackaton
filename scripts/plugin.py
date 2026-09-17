"""Load a swappable implementation named "module:Name", where module is importable or a .py path."""

from __future__ import annotations

import importlib
import importlib.util
from pathlib import Path


def load(spec: str, builtin: dict[str, str] | None = None):
    spec = (builtin or {}).get(spec, spec)
    module_name, _, name = spec.rpartition(":")
    if not module_name:
        raise ValueError(f"{spec!r} is not a built-in name or module:Name")
    if module_name.endswith(".py"):
        location = importlib.util.spec_from_file_location(Path(module_name).stem, module_name)
        module = importlib.util.module_from_spec(location)
        location.loader.exec_module(module)
    else:
        module = importlib.import_module(module_name)
    return getattr(module, name)
