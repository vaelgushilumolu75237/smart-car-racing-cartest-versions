"""Run the r24 algorithm tests and report historical/platform-only exclusions.

Do not alter baseline tests to conceal failures. The explicit exclusions refer
to old OpenMV package installation variants and an AST freeze against pre-port
imports; their contracts are replaced by K230 runtime/manifest and HAL checks.
"""
from pathlib import Path
import json
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
EXCLUDED = {
    "test_profile_packages.ProfilePackageTests.test_all_three_manifests_cover_complete_profile_matched_sources": "Historical OpenMV no_pd/pd/pd_preview delivery manifests, not K230 installation packages",
    "test_profile_packages.ProfilePackageTests.test_profile_identity_is_unique_and_other_runtime_sources_are_shared": "Historical OpenMV no_pd/pd/pd_preview package identity, replaced by K230 manifest",
    "test_steering_diag.SteeringDiagAlgorithmFreezeTests.test_unrelated_vision_ast_matches_baseline_after_control_changes": "ARM-to-RISC-V port intentionally changes imports, decorators and camera/UART setup; upstream geometry regression remains",
}


def flatten(suite):
    for item in suite:
        if isinstance(item, unittest.TestSuite):
            yield from flatten(item)
        else:
            yield item


if __name__ == "__main__":
    sys.path.insert(0, str(ROOT))
    sys.path.insert(0, str(ROOT / "openmv"))
    suite = unittest.defaultTestLoader.discover(str(ROOT / "openmv" / "tests"), pattern="test_*.py")
    selected, excluded = [], {}
    for case in flatten(suite):
        reason = EXCLUDED.get(case.id())
        if reason:
            excluded[case.id()] = reason
        else:
            selected.append(case)
    if set(excluded) != set(EXCLUDED):
        print("DISCOVERY_MISSING_EXCLUSIONS", sorted(set(EXCLUDED) - set(excluded)))
        for case in selected:
            if "_FailedTest" in case.id():
                print(case.id(), str(getattr(case, "_exception", "")))
        raise SystemExit("Exclusion identity changed; inspect the actual tests")
    result = unittest.TextTestRunner(verbosity=1).run(unittest.TestSuite(selected))
    out = ROOT.parent / "validation"; out.mkdir(exist_ok=True)
    (out / "CAMERA_TESTS.json").write_text(json.dumps({
        "run": result.testsRun, "failures": len(result.failures),
        "errors": len(result.errors), "skipped": [(case.id(), reason) for case, reason in result.skipped],
        "excluded": excluded, "hardware": "not_connected",
    }, indent=2), encoding="utf-8")
    sys.exit(0 if result.wasSuccessful() else 1)
