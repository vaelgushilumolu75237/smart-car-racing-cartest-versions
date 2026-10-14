"""Save a source/ELF checkpoint and optionally publish it to the owner's GitHub.

Credentials are obtained from Git Credential Manager in memory and never logged.
The checkpoint worktree is separate from the live CarTest source directory.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import urllib.error
import urllib.request

PROJECT = Path(__file__).resolve().parents[1]
STORE = PROJECT / "deliverables" / "github-checkpoints" / "worktree"
REPO_NAME = "smart-car-racing-cartest-versions"
DIRS = ("App", "Inc", "Src", "Drivers", "cmake", "tools", "tests", "docs", ".vscode", "openmv")
ROOT_FILES = ("CMakeLists.txt", "CMakePresets.json", "CarTest.code-workspace",
              "project-description.json", "stm32h743xi_flash.ld", "README.md", "AGENTS.md",
              "README-恢复说明.md", "origin-manifest.json", "SHA256SUMS.txt")
SKIP_DIRS = {"__pycache__", ".git", "logs", "tmp", "node_modules"}
SKIP_SUFFIXES = {".pyc", ".obj", ".o", ".exe", ".dll", ".png", ".jpg",
                 ".jpeg", ".mp4", ".avi", ".bin", ".log"}


def run_git(*args: str) -> str:
    env = dict(os.environ, GIT_TERMINAL_PROMPT="0", GCM_INTERACTIVE="never")
    result = subprocess.run(["git", "-C", str(STORE), *args], capture_output=True,
                            text=True, encoding="utf-8", env=env, timeout=180)
    if result.returncode:
        raise RuntimeError(f"git {args[0]} failed ({result.returncode}): {result.stderr.strip()}")
    return result.stdout.strip()


def api(path: str, token: str, body=None):
    headers = {"Authorization": "Bearer " + token,
               "User-Agent": "CarTest-version-checkpoint",
               "Accept": "application/vnd.github+json"}
    request = urllib.request.Request("https://api.github.com" + path, headers=headers,
                                    data=None if body is None else json.dumps(body).encode())
    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            return json.load(response)
    except urllib.error.HTTPError as error:
        if error.code == 404 and body is None:
            return None
        raise RuntimeError(f"GitHub API {path}: HTTP {error.code}") from None


def authenticated_user():
    env = dict(os.environ, GIT_TERMINAL_PROMPT="0", GCM_INTERACTIVE="never")
    result = subprocess.run(["git", "credential", "fill"],
                            input="protocol=https\nhost=github.com\n\n", capture_output=True,
                            text=True, env=env, timeout=30)
    if result.returncode:
        raise RuntimeError("Git Credential Manager has no usable noninteractive GitHub credential.")
    credential = dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)
    token = credential.get("password", "")
    user = api("/user", token)
    if not user or not user.get("login"):
        raise RuntimeError("GitHub account verification failed.")
    return user, token


def safe_remove(path: Path):
    resolved = path.resolve()
    if resolved.parent != STORE.resolve() or resolved.name != "CarTest":
        raise RuntimeError("Refusing to clear a directory outside the checkpoint worktree.")
    if resolved.exists():
        shutil.rmtree(resolved)


def copy_tree(source: Path, destination: Path):
    for item in sorted(source.rglob("*")):
        relative = item.relative_to(source)
        if any(part in SKIP_DIRS for part in relative.parts):
            continue
        if item.is_symlink():
            raise RuntimeError(f"Symlink is not allowed in a public source checkpoint: {relative}")
        if item.is_file() and item.suffix.lower() not in SKIP_SUFFIXES:
            target = destination / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(item, target)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=PROJECT)
    parser.add_argument("--version", required=True)
    parser.add_argument("--branch", required=True)
    parser.add_argument("--status", required=True)
    parser.add_argument("--publish", action="store_true")
    args = parser.parse_args()
    for value in (args.version, args.branch):
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._/-]*", value) or ".." in value or value.endswith("/"):
            raise ValueError("Use a plain version/branch name without traversal or Git special syntax.")
    source = args.source.resolve()
    if source == STORE.resolve() or STORE.resolve() in source.parents:
        raise ValueError("The source must be outside the checkpoint worktree.")
    if not (source / "Src" / "main.c").is_file() or not (source / "openmv" / "main.py").is_file():
        raise ValueError("A complete CarTest H743/OpenMV source tree is required.")
    presets = json.loads((source / "CMakePresets.json").read_text(encoding="utf-8-sig"))
    profile_names = sorted(p["name"] for p in presets.get("buildPresets", [])
                           if not p.get("hidden", False))
    for profile in profile_names:
        if not (source / "build" / profile / "CarTest.elf").is_file():
            raise RuntimeError(f"Build the declared profile before checkpointing: {profile}")
    STORE.mkdir(parents=True, exist_ok=True)
    if not (STORE / ".git").is_dir():
        run_git("init", "-b", "main")
    run_git("config", "core.autocrlf", "false")
    if run_git("status", "--porcelain"):
        raise RuntimeError("Checkpoint worktree has uncommitted changes; preserve them before retrying.")
    existing = run_git("for-each-ref", "--format=%(refname:short)", "refs/heads").splitlines()
    if args.branch in existing:
        run_git("switch", args.branch)
    elif existing:
        run_git("switch", "-c", args.branch)
    elif args.branch != "main":
        run_git("symbolic-ref", "HEAD", "refs/heads/" + args.branch)
    if run_git("tag", "--list", args.version):
        raise RuntimeError("This immutable version tag already exists; choose a new version.")
    target = STORE / "CarTest"
    safe_remove(target)
    target.mkdir()
    for directory in DIRS:
        if (source / directory).is_dir():
            copy_tree(source / directory, target / directory)
    for name in ROOT_FILES:
        if (source / name).is_file():
            shutil.copy2(source / name, target / name)
    for profile in profile_names:
        elf = source / "build" / profile / "CarTest.elf"
        output = target / "build" / elf.parent.name / elf.name
        output.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(elf, output)
    manifest = {str(p.relative_to(target)).replace("\\", "/"):
                hashlib.sha256(p.read_bytes()).hexdigest()
                for p in sorted(target.rglob("*")) if p.is_file()}
    (STORE / "SHA256SUMS.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    (STORE / "VERSION.json").write_text(json.dumps({"version": args.version,
        "status": args.status, "branch": args.branch, "files": len(manifest),
        "hardware_actions": "none"}, indent=2) + "\n", encoding="utf-8")
    (STORE / "README.md").write_text("# CarTest version checkpoints\n\n"
        "H743 source, OpenMV upload files, and paired ELF checkpoints.\n\n"
        "Select a tag to restore a complete version. Experimental versions are not vehicle acceptance.\n"
        "The live source directory is not modified by this checkpoint tool.\n", encoding="utf-8")
    (STORE / ".gitattributes").write_text("* -text\n", encoding="utf-8")
    user, token = authenticated_user() if args.publish else ({"login": "CarTest-checkpoint", "id": 0}, "")
    run_git("-c", "user.name=" + user["login"], "-c",
            f"user.email={user['id']}+{user['login']}@users.noreply.github.com",
            "add", "--all")
    run_git("-c", "user.name=" + user["login"], "-c",
            f"user.email={user['id']}+{user['login']}@users.noreply.github.com",
            "commit", "-m", f"Checkpoint {args.version}: {args.status}")
    run_git("tag", args.version)
    commit = run_git("rev-parse", "HEAD")
    if args.publish:
        repo = api(f"/repos/{user['login']}/{REPO_NAME}", token)
        if repo is None:
            repo = api("/user/repos", token, {"name": REPO_NAME, "private": False,
                "description": "STM32H743 CarTest and OpenMV version checkpoints", "auto_init": False})
        if repo.get("private") or repo.get("owner", {}).get("login") != user["login"]:
            raise RuntimeError("Repository visibility/ownership does not match the authorized public backup.")
        remote = repo["clone_url"]
        if "origin" in run_git("remote").splitlines():
            if run_git("remote", "get-url", "origin") != remote:
                raise RuntimeError("The existing origin differs from the authorized backup repository.")
        else:
            run_git("remote", "add", "origin", remote)
        run_git("push", "--atomic", "origin", args.branch, "refs/tags/" + args.version)
        remote_commit = run_git("ls-remote", "origin", "refs/tags/" + args.version).split()[0]
        if remote_commit != commit:
            raise RuntimeError("Remote version verification failed.")
        print(json.dumps({"repository": repo["html_url"], "version": args.version,
                          "commit": commit, "remote_verified": True, "files": len(manifest)}))
    else:
        print(json.dumps({"version": args.version, "commit": commit, "published": False}))


if __name__ == "__main__":
    main()
