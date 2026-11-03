"""Architecture rules, enforced by reading the import statements of every module.

The central rule is the *wall of truth*: the robot's software (belief side)
must never import the simulator's ground truth, otherwise "autonomy" could
quietly cheat. See docs/ARCHITECTURE.md, section 2.
"""

from __future__ import annotations

import ast
from functools import cache
from pathlib import Path

import pytest

import roombacalypse

PACKAGE_ROOT = Path(roombacalypse.__file__).parent
PACKAGE = "roombacalypse"

# Every subpackage must be classified; adding one forces a deliberate decision.
LAYERS: dict[str, str] = {
    "core": "shared",
    "environment": "truth",
    "robot": "truth",
    "sensors": "truth",
    "simulation": "truth",
    "cleaning": "truth",
    "battery": "truth",
    "mapping": "belief",
    "localization": "belief",
    "planning": "belief",
    "navigation": "belief",
    "autonomy": "belief",
    "personality": "observer",
    "evaluation": "harness",
    "visualization": "harness",
}
ALLOWED: dict[str, set[str]] = {
    "shared": {"shared"},
    "truth": {"shared", "truth"},
    "belief": {"shared", "belief"},
    "observer": {"shared"},
    "harness": {"shared", "truth", "belief", "observer", "harness"},
}
TOP_LEVEL_MODULES = {"__init__", "__main__", "app"}  # composition root: may import anything


def _modules() -> list[Path]:
    return sorted(PACKAGE_ROOT.rglob("*.py"))


def _module_name(path: Path) -> str:
    parts = path.relative_to(PACKAGE_ROOT.parent).with_suffix("").parts
    return ".".join(parts[:-1] if parts[-1] == "__init__" else parts)


@cache
def _imports(path: Path) -> frozenset[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    current = _module_name(path)
    is_package = path.name == "__init__.py"
    found: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            found.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level:  # resolve relative imports
                base = current.split(".")
                base = base if is_package else base[:-1]
                base = base[: len(base) - (node.level - 1)]
                module = ".".join(base + ([node.module] if node.module else []))
            else:
                module = node.module or ""
            found.add(module)
    return frozenset(found)


def _subpackage(module: str) -> str | None:
    parts = module.split(".")
    if parts[0] != PACKAGE or len(parts) < 2:
        return None
    return parts[1]


@pytest.mark.parametrize("path", _modules(), ids=lambda p: str(p.relative_to(PACKAGE_ROOT)))
def test_layering(path: Path) -> None:
    source = _subpackage(_module_name(path))
    if source is None or source in TOP_LEVEL_MODULES:
        return
    assert source in LAYERS, f"subpackage '{source}' is not classified in tests/test_architecture.py"
    for imported in _imports(path):
        target = _subpackage(imported)
        if target is None:
            continue  # stdlib, third party, or the package root (e.g. __version__)
        assert target not in TOP_LEVEL_MODULES, f"{_module_name(path)} must not import the composition root {imported}"
        assert target in LAYERS, f"{path.name} imports unclassified '{imported}'"
        assert LAYERS[target] in ALLOWED[LAYERS[source]], (
            f"{_module_name(path)} ({LAYERS[source]}) must not import {imported} ({LAYERS[target]})"
        )


@pytest.mark.parametrize("path", _modules(), ids=lambda p: str(p.relative_to(PACKAGE_ROOT)))
def test_only_visualization_imports_pygame(path: Path) -> None:
    if _subpackage(_module_name(path)) == "visualization":
        return
    offenders = {m for m in _imports(path) if m == "pygame" or m.startswith("pygame.")}
    assert not offenders, f"{path} imports pygame; rendering belongs in roombacalypse.visualization"


def test_no_import_cycles_between_subpackages() -> None:
    graph: dict[str, set[str]] = {}
    for path in _modules():
        source = _subpackage(_module_name(path))
        if source is None or source in TOP_LEVEL_MODULES:
            continue
        for imported in _imports(path):
            target = _subpackage(imported)
            if target and target != source and target not in TOP_LEVEL_MODULES:
                graph.setdefault(source, set()).add(target)

    visiting: set[str] = set()
    done: set[str] = set()

    def visit(node: str, trail: list[str]) -> None:
        if node in done:
            return
        assert node not in visiting, f"import cycle: {' -> '.join([*trail, node])}"
        visiting.add(node)
        for nxt in sorted(graph.get(node, ())):
            visit(nxt, [*trail, node])
        visiting.discard(node)
        done.add(node)

    for node in sorted(graph):
        visit(node, [])


@pytest.mark.parametrize("path", _modules(), ids=lambda p: str(p.relative_to(PACKAGE_ROOT)))
def test_library_code_logs_instead_of_printing(path: Path) -> None:
    if _module_name(path) in (f"{PACKAGE}.app", f"{PACKAGE}.__main__"):
        return
    tree = ast.parse(path.read_text(encoding="utf-8"))
    prints = [
        node.lineno
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "print"
    ]
    assert not prints, f"{path}: print() on line(s) {prints}; use logging"
