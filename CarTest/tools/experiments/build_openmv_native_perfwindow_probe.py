#!/usr/bin/env python3
"""Build and validate a RAM-VFS candidate with native perf-window methods.

This is an isolated experiment. It does not edit or package production files.
The candidate uses the coordinated production profile and decorates only
_V7PerfWindow.clear, record, and _stage_summary with literal native directives.
"""

from __future__ import annotations

import argparse
import ast
from array import array
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import zlib


PROJECT_ROOT = Path(__file__).resolve().parents[2]
TOOLS_ROOT = PROJECT_ROOT / "tools"
if str(TOOLS_ROOT) not in sys.path:
    sys.path.insert(0, str(TOOLS_ROOT))

from build_openmv_ackermann320_packages import (  # noqa: E402
    COMPILER as DEFAULT_COMPILER,
    OPENMV_DIR,
    PACKAGES,
    package_source_text,
)
from build_openmv_ram_preview_probe import (  # noqa: E402
    RAM_VFS_PREFIX,
    RUNTIME_MODULES,
)


OUTPUT_ROOT = (PROJECT_ROOT / "logs" / "usb-preview-20261024" /
               "native-perfwindow")
DEFAULT_RAM_SOURCE = OUTPUT_ROOT / "native-perfwindow-ram-source.py"
DEFAULT_CANDIDATE_MAIN = OUTPUT_ROOT / "main-native-perfwindow.py"
DEFAULT_CANDIDATE_MPY = OUTPUT_ROOT / "main-native-perfwindow.mpy"
DEFAULT_MANIFEST = OUTPUT_ROOT / "manifest.json"
TARGET = "armv7emsp"
NATIVE_METHODS = ("clear", "record", "_stage_summary")

RAM_PROBE_SUFFIX = '''
 for _name in RUNTIME_MODULES_PLACEHOLDER:
  if _name in sys.modules:del sys.modules[_name]
  __import__(_name)
  gc.collect()
 import main,main_baseline
 _STAGES=main._V7_PERF_STAGES
 _SAMPLES=(
     ("capture",-1),("masks",999),("masks",1000),
     ("path_topology",201000),("row_weight",201001),
     ("overlay",2147483647),("overlay",2147483649),
     ("diag_trace",3000000000),("total_cycle",5000000000))
 def _state(window):
  return tuple((stage,tuple(window.bins[stage]),window.sums[stage],
                window.maxima[stage],window.counts[stage])
               for stage in _STAGES)+(
       ("frames",window.frames),("valid_frames",window.valid_frames),
       ("valid_full_writes",window.valid_full_writes),
       ("late_invalid_frames",window.late_invalid_frames),
       ("write_attempts",window.write_attempts),
       ("full_writes",window.full_writes),
       ("write_errors",window.write_errors),
       ("last_sequence",window.last_sequence))
 def _aliases(window):
  return (window.bins,window.sums,window.maxima,window.counts,
          tuple(window.bins[stage] for stage in _STAGES))
 def _aliases_preserved(window,aliases):
  return (window.bins is aliases[0] and window.sums is aliases[1] and
          window.maxima is aliases[2] and window.counts is aliases[3] and
          all(window.bins[stage] is aliases[4][index]
              for index,stage in enumerate(_STAGES)))
 def _summary(window):
  return (tuple(window._stage_summary(stage) for stage in _STAGES),
          window.summary_line(20261024,0,0),
          tuple(window.summary_lines(20261024,0,0)))
 _old=main_baseline._V7PerfWindow()
 _new=main._V7PerfWindow()
 _old_aliases=_aliases(_old)
 _new_aliases=_aliases(_new)
 _empty_old=_summary(_old)
 _empty_new=_summary(_new)
 for _stage,_value in _SAMPLES:
  _old.record(_stage,_value)
  _new.record(_stage,_value)
 _fixed_old=_summary(_old)
 _fixed_new=_summary(_new)
 _equal=(_state(_old)==_state(_new) and
         _empty_old==_empty_new and _fixed_old==_fixed_new)
 _alias_ok=(_aliases_preserved(_old,_old_aliases) and
            _aliases_preserved(_new,_new_aliases))
 print("PERFWINDOW_FIXED_DIFF equal=%d aliases_preserved=%d samples=%d "
       "large_sum=%d" % (1 if _equal else 0,1 if _alias_ok else 0,
       len(_SAMPLES),_new.sums["overlay"]+_new.sums["diag_trace"]+
       _new.sums["total_cycle"]))
 if not _equal or not _alias_ok:
  raise RuntimeError("native perf-window fixed-sample differential failed")
 _old.clear()
 _new.clear()
 if (_state(_old)!=_state(_new) or
     not _aliases_preserved(_old,_old_aliases) or
     not _aliases_preserved(_new,_new_aliases)):
  raise RuntimeError("native perf-window clear/alias differential failed")
 print("PERFWINDOW_CLEAR_DIFF equal=1 aliases_preserved=1")
 def _record_bench(window,rounds):
  _values=(400,1200,2400,600,1800,3100,900,1500,700,2200,
           500,1300,2900,800,1700,1100,2600,1000,2000)
  _start=time.ticks_us()
  for _round in range(rounds):
   for _index in range(19):
    window.record(_STAGES[_index%len(_STAGES)],_values[_index])
  return time.ticks_diff(time.ticks_us(),_start)
 _old.clear()
 _new.clear()
 _record_rounds=300
 _old_record_us=_record_bench(_old,_record_rounds)
 _new_record_us=_record_bench(_new,_record_rounds)
 if _state(_old)!=_state(_new):
  raise RuntimeError("native perf-window repeated record state diverged")
 print("PERFWINDOW_TIMING record_calls=%d old_us=%d native_us=%d" %
       (_record_rounds*19,_old_record_us,_new_record_us))
 _clear_rounds=3
 _start=time.ticks_us()
 for _ in range(_clear_rounds):_old.clear()
 _old_clear_us=time.ticks_diff(time.ticks_us(),_start)
 _start=time.ticks_us()
 for _ in range(_clear_rounds):_new.clear()
 _new_clear_us=time.ticks_diff(time.ticks_us(),_start)
 if (_state(_old)!=_state(_new) or
     not _aliases_preserved(_old,_old_aliases) or
     not _aliases_preserved(_new,_new_aliases)):
  raise RuntimeError("native perf-window repeated clear diverged")
 print("PERFWINDOW_TIMING clear_calls=%d old_us=%d native_us=%d" %
       (_clear_rounds,_old_clear_us,_new_clear_us))
 _old.record("capture",999)
 _new.record("capture",999)
 if _summary(_old)!=_summary(_new):
  raise RuntimeError("native perf-window stage summary diverged")
 _summary_rounds=50
 _start=time.ticks_us()
 for _ in range(_summary_rounds):_old._stage_summary("capture")
 _old_summary_us=time.ticks_diff(time.ticks_us(),_start)
 _start=time.ticks_us()
 for _ in range(_summary_rounds):_new._stage_summary("capture")
 _new_summary_us=time.ticks_diff(time.ticks_us(),_start)
 print("PERFWINDOW_TIMING stage_summary_calls=%d old_us=%d native_us=%d" %
       (_summary_rounds,_old_summary_us,_new_summary_us))
 print("NATIVE_PERFWINDOW_PROBE_DONE")
except Exception as exc:
 sys.print_exception(exc)
 print("NATIVE_PERFWINDOW_PROBE_FAILED")
'''


def _find_perf_class(source: str) -> tuple[ast.Assign, ast.ClassDef, ast.Module]:
    tree = ast.parse(source)
    stage_assignments = []
    perf_classes = []
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                isinstance(target, ast.Name) and
                target.id == "_V7_PERF_STAGES"
                for target in node.targets):
            stage_assignments.append(node)
        if isinstance(node, ast.ClassDef) and node.name == "_V7PerfWindow":
            perf_classes.append(node)
    if len(stage_assignments) != 1 or len(perf_classes) != 1:
        raise RuntimeError("expected one perf-stage assignment and class")
    return stage_assignments[0], perf_classes[0], tree


def decorate_perf_methods(source: str) -> str:
    """Add literal decorators to exactly the three selected class methods."""
    _, perf_class, _ = _find_perf_class(source)
    methods = {
        node.name: node for node in perf_class.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    }
    if any(name not in methods for name in NATIVE_METHODS):
        raise RuntimeError("one or more selected perf methods are missing")
    lines = source.splitlines(keepends=True)
    insertions = []
    for name in NATIVE_METHODS:
        node = methods[name]
        for decorator in node.decorator_list:
            if (isinstance(decorator, ast.Attribute) and
                    isinstance(decorator.value, ast.Name) and
                    decorator.value.id == "micropython" and
                    decorator.attr == "native"):
                raise RuntimeError("selected method already has native decorator")
        definition = lines[node.lineno - 1]
        indent = definition[:len(definition) - len(definition.lstrip())]
        insertions.append((node.lineno - 1,
                           indent + "@micropython.native\n"))
    for line_index, decorator_line in sorted(insertions, reverse=True):
        lines.insert(line_index, decorator_line)
    candidate = "".join(lines)
    compile(candidate, "main-native-perfwindow.py", "exec")
    _, decorated_class, _ = _find_perf_class(candidate)
    decorated = {
        node.name: node for node in decorated_class.body
        if isinstance(node, ast.FunctionDef)
    }
    for name in NATIVE_METHODS:
        decorators = decorated[name].decorator_list
        if not any(isinstance(item, ast.Attribute) and
                   isinstance(item.value, ast.Name) and
                   item.value.id == "micropython" and
                   item.attr == "native" for item in decorators):
            raise RuntimeError("literal native decorator missing for " + name)
    return candidate


class _NativeHostCompat:
    @staticmethod
    def native(function):
        return function


def _load_perf_class(source: str):
    stage_assignment, perf_class, _ = _find_perf_class(source)
    snippet = "\n\n".join((
        ast.get_source_segment(source, stage_assignment),
        ast.get_source_segment(source, perf_class),
    ))
    namespace = {
        "array": array,
        "micropython": _NativeHostCompat,
        "config": types.SimpleNamespace(PERF_WINDOW_MS=10000),
    }
    exec(compile(snippet, "perfwindow_host_diff.py", "exec"), namespace)
    return namespace["_V7PerfWindow"], namespace["_V7_PERF_STAGES"]


def _perf_state(window, stages) -> tuple:
    state = [
        (stage, tuple(window.bins[stage]), window.sums[stage],
         window.maxima[stage], window.counts[stage])
        for stage in stages
    ]
    state.extend((
        ("frames", window.frames),
        ("valid_frames", window.valid_frames),
        ("valid_full_writes", window.valid_full_writes),
        ("late_invalid_frames", window.late_invalid_frames),
        ("write_attempts", window.write_attempts),
        ("full_writes", window.full_writes),
        ("write_errors", window.write_errors),
        ("last_sequence", window.last_sequence),
    ))
    return tuple(state)


def _perf_aliases(window, stages) -> tuple:
    return (window.bins, window.sums, window.maxima, window.counts,
            tuple(window.bins[stage] for stage in stages))


def _perf_aliases_preserved(window, aliases, stages) -> bool:
    return (window.bins is aliases[0] and window.sums is aliases[1] and
            window.maxima is aliases[2] and window.counts is aliases[3] and
            all(window.bins[stage] is aliases[4][index]
                for index, stage in enumerate(stages)))


def _perf_summaries(window, stages) -> tuple:
    return (
        tuple(window._stage_summary(stage) for stage in stages),
        window.summary_line(20261024, 0, 0),
        tuple(window.summary_lines(20261024, 0, 0)),
    )


def run_host_differential(baseline_source: str,
                          candidate_source: str) -> dict:
    old_class, old_stages = _load_perf_class(baseline_source)
    new_class, new_stages = _load_perf_class(candidate_source)
    if old_stages != new_stages:
        raise AssertionError("stage names or order changed")
    stages = old_stages
    old = old_class()
    new = new_class()
    old_aliases = _perf_aliases(old, stages)
    new_aliases = _perf_aliases(new, stages)
    empty_old = _perf_summaries(old, stages)
    empty_new = _perf_summaries(new, stages)
    if empty_old != empty_new:
        raise AssertionError("empty summary changed")

    samples = (
        ("capture", -1),
        ("masks", 0),
        ("masks", 999),
        ("masks", 1000),
        ("masks", 1999),
        ("path_topology", 201000),
        ("row_weight", 201001),
        ("overlay", 2147483647),
        ("overlay", 2147483649),
        ("diag_trace", 3000000000),
        ("total_cycle", 5000000000),
        ("total_cycle", -2147483648),
    )
    for stage, value in samples:
        old.record(stage, value)
        new.record(stage, value)
    if _perf_state(old, stages) != _perf_state(new, stages):
        raise AssertionError("record state changed")
    if _perf_summaries(old, stages) != _perf_summaries(new, stages):
        raise AssertionError("non-empty summary changed")
    if old.bins["capture"][0] != 1:
        raise AssertionError("negative values no longer clamp to bucket 0")
    if new.bins["masks"][0] != 2 or new.bins["masks"][1] != 2:
        raise AssertionError("999/1000 bucket boundaries changed")
    if new.bins["path_topology"][201] != 1 or \
            new.bins["row_weight"][201] != 1:
        raise AssertionError("overflow bucket boundary changed")
    if new.sums["overlay"] <= 0xFFFFFFFF:
        raise AssertionError("large sum did not exceed unsigned 32-bit range")
    if not _perf_aliases_preserved(old, old_aliases, stages) or \
            not _perf_aliases_preserved(new, new_aliases, stages):
        raise AssertionError("public dict/array identity changed after record")

    old.clear()
    new.clear()
    if _perf_state(old, stages) != _perf_state(new, stages):
        raise AssertionError("clear state changed")
    if _perf_summaries(old, stages) != _perf_summaries(new, stages):
        raise AssertionError("cleared summaries changed")
    if not _perf_aliases_preserved(old, old_aliases, stages) or \
            not _perf_aliases_preserved(new, new_aliases, stages):
        raise AssertionError("clear replaced public dict/array aliases")
    if any(any(bins) for bins in new.bins.values()):
        raise AssertionError("clear left histogram bins nonzero")
    return {
        "status": "PASS",
        "native_methods": list(NATIVE_METHODS),
        "stage_count": len(stages),
        "fixed_samples": len(samples),
        "negative_clamp": "PASS",
        "buckets_999_1000_201000_plus": "PASS",
        "large_value_and_sum": "PASS",
        "empty_and_clear_summaries": "PASS",
        "dict_and_array_identity": "PASS",
    }


def _coordinated_package() -> dict:
    matches = [package for package in PACKAGES
               if package["name"] == "coordinated"]
    if len(matches) != 1:
        raise RuntimeError("coordinated package profile is unavailable")
    return matches[0]


def _packaged_source(name: str, package: dict) -> str:
    filename = name if name.endswith(".py") else name + ".py"
    return package_source_text(OPENMV_DIR / filename, package)


def build_ram_source(candidate_main: str, baseline_main: str,
                     package: dict) -> str:
    parts = [RAM_VFS_PREFIX.rstrip("\r\n")]
    for name in RUNTIME_MODULES:
        source = (candidate_main if name == "main" else
                  _packaged_source(name, package))
        parts.append(" _ram_sources[%r]=%r" % (
            name + ".py", zlib.compress(source.encode("utf-8"))))
    parts.append(" _ram_sources[%r]=%r" % (
        "main_baseline.py",
        zlib.compress(baseline_main.encode("utf-8"))))
    suffix = RAM_PROBE_SUFFIX.replace(
        "RUNTIME_MODULES_PLACEHOLDER",
        repr(list(RUNTIME_MODULES) + ["main_baseline"]))
    parts.append(suffix.strip("\r\n"))
    source = "\n".join(parts) + "\n"
    source = source.replace("\r\n", "\n").replace("\r", "\n")
    source = source.replace("\n", "\r\n")
    compile(source, "native_perfwindow_ram_probe.py", "exec")
    return source


def cross_compile(candidate_source: str, compiler: Path,
                  output_mpy: Path) -> dict:
    compiler = Path(compiler)
    if not compiler.is_file():
        raise FileNotFoundError("mpy-cross compiler not found: %s" % compiler)
    version = subprocess.run(
        [str(compiler), "--version"], check=True,
        capture_output=True, text=True)
    with tempfile.TemporaryDirectory(prefix="native-perfwindow-") as temp:
        work = Path(temp)
        source_path = work / "main.py"
        candidate_path = work / "main.mpy"
        source_path.write_text(candidate_source, encoding="utf-8", newline="")
        command = [str(compiler), "-march=" + TARGET, "-s", "main.py",
                   "-o", str(candidate_path), str(source_path)]
        result = subprocess.run(command, capture_output=True, text=True)
        if result.returncode:
            raise RuntimeError(
                "mpy-cross candidate compile failed (%d):\n%s\n%s" % (
                    result.returncode, result.stdout, result.stderr))
        payload = candidate_path.read_bytes()
    output_mpy.parent.mkdir(parents=True, exist_ok=True)
    output_mpy.write_bytes(payload)
    return {
        "compiler": str(compiler),
        "compiler_version": version.stdout.strip() or version.stderr.strip(),
        "target": TARGET,
        "command": ["mpy-cross", "-march=" + TARGET, "-s", "main.py"],
        "mpy_bytes": len(payload),
        "mpy_sha256": hashlib.sha256(payload).hexdigest(),
    }


def build(output_root: Path = OUTPUT_ROOT,
          compiler: Path = DEFAULT_COMPILER) -> dict:
    package = _coordinated_package()
    baseline_main = _packaged_source("main.py", package)
    candidate_main = decorate_perf_methods(baseline_main)
    compile(candidate_main, "main-native-perfwindow.py", "exec")
    host_diff = run_host_differential(baseline_main, candidate_main)
    ram_source = build_ram_source(candidate_main, baseline_main, package)

    output_root = Path(output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    candidate_main_path = output_root / DEFAULT_CANDIDATE_MAIN.name
    ram_source_path = output_root / DEFAULT_RAM_SOURCE.name
    candidate_mpy_path = output_root / DEFAULT_CANDIDATE_MPY.name
    manifest_path = output_root / DEFAULT_MANIFEST.name
    candidate_main_path.write_text(candidate_main, encoding="utf-8",
                                   newline="")
    ram_payload = ram_source.encode("utf-8")
    ram_source_path.write_bytes(ram_payload)
    compile_info = cross_compile(candidate_main, compiler, candidate_mpy_path)
    manifest = {
        "experiment": "native_v7_perfwindow_methods",
        "production_modified": False,
        "package_profile": package["name"],
        "target": TARGET,
        "methods_decorated": list(NATIVE_METHODS),
        "host_differential": host_diff,
        "cross_compile": compile_info,
        "candidate_main": {
            "path": candidate_main_path.name,
            "bytes": candidate_main_path.stat().st_size,
            "sha256": hashlib.sha256(candidate_main_path.read_bytes()).hexdigest(),
        },
        "ram_source": {
            "path": ram_source_path.name,
            "bytes": len(ram_payload),
            "sha256": hashlib.sha256(ram_payload).hexdigest(),
        },
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) +
                             "\n", encoding="utf-8", newline="")
    return manifest


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=OUTPUT_ROOT)
    parser.add_argument("--compiler", type=Path, default=DEFAULT_COMPILER)
    args = parser.parse_args(argv)
    manifest = build(args.output_root, args.compiler)
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
