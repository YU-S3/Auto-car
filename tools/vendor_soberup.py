"""Download and verify a pinned upstream source snapshot; never execute it."""

from __future__ import annotations

import hashlib
import io
import json
from pathlib import Path, PurePosixPath
import stat
import urllib.request
import zipfile


REPOSITORY = "Mantou-WYH/soberup-2026-public"
COMMIT = "7abfdac0d0156576ba5186716a7c1e42992359ca"
EXPECTED_TREE = "0a17400d274d55768758f326566368ad1c4b23c6"
ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "soberup"
UPSTREAM = BASE / "upstream"
ARCHIVE = BASE / "archive.zip"
MAX_ARCHIVE = 32 * 1024 * 1024
MAX_EXPANDED = 128 * 1024 * 1024
SOURCE_URL = f"https://codeload.github.com/{REPOSITORY}/zip/{COMMIT}"


def git_hash(kind: str, data: bytes) -> bytes:
    return hashlib.sha1(kind.encode() + b" " + str(len(data)).encode() + b"\0" + data).digest()


def calculate_tree(entries: list[dict]) -> str:
    root: dict = {}
    for entry in entries:
        node = root
        parts = entry["path"].split("/")
        for part in parts[:-1]:
            node = node.setdefault(part, {})
        if parts[-1] in node:
            raise ValueError("Duplicate archive path: " + entry["path"])
        node[parts[-1]] = (entry["git_mode"], bytes.fromhex(entry["git_blob_sha1"]))

    def visit(node: dict) -> bytes:
        items = []
        for name, value in node.items():
            is_directory = isinstance(value, dict)
            mode, digest = ("40000", visit(value)) if is_directory else value
            key = name.encode("utf-8") + (b"/" if is_directory else b"")
            items.append((key, mode.encode() + b" " + name.encode("utf-8") + b"\0" + digest))
        return git_hash("tree", b"".join(body for _, body in sorted(items)))

    return visit(root).hex()


def load_archive() -> bytes:
    if ARCHIVE.exists():
        if ARCHIVE.stat().st_size > MAX_ARCHIVE:
            raise ValueError("Existing archive exceeds the download limit")
        return ARCHIVE.read_bytes()
    request = urllib.request.Request(SOURCE_URL, headers={"User-Agent": "InteligentAutomove-baseline"})
    with urllib.request.urlopen(request, timeout=30) as response:
        payload = response.read(MAX_ARCHIVE + 1)
    if len(payload) > MAX_ARCHIVE:
        raise ValueError("Archive exceeds the download limit")
    return payload


def unpack_verified(payload: bytes) -> list[tuple[dict, bytes]]:
    result = []
    prefix = f"soberup-2026-public-{COMMIT}"
    with zipfile.ZipFile(io.BytesIO(payload)) as archive:
        if sum(item.file_size for item in archive.infolist()) > MAX_EXPANDED:
            raise ValueError("Expanded source exceeds the size limit")
        for item in archive.infolist():
            path = PurePosixPath(item.filename)
            if path.is_absolute() or ".." in path.parts or not path.parts or path.parts[0] != prefix:
                raise ValueError("Unexpected archive path: " + item.filename)
            if item.is_dir():
                continue
            relative = PurePosixPath(*path.parts[1:])
            if not relative.parts or any(":" in part or "\\" in part for part in relative.parts):
                raise ValueError("Invalid source path: " + item.filename)
            mode = item.external_attr >> 16
            if stat.S_ISLNK(mode):
                raise ValueError("Symbolic links are not accepted")
            data = archive.read(item)  # zipfile also checks the member CRC.
            record = {
                "path": relative.as_posix(),
                "bytes": len(data),
                "git_mode": "100755" if mode & 0o111 else "100644",
                "git_blob_sha1": git_hash("blob", data).hex(),
                "sha256": hashlib.sha256(data).hexdigest(),
            }
            result.append((record, data))
    actual_tree = calculate_tree([record for record, _ in result])
    if actual_tree != EXPECTED_TREE:
        raise ValueError(f"Git tree mismatch: {actual_tree} != {EXPECTED_TREE}")
    return result


def write_once_or_verify(path: Path, data: bytes) -> None:
    if path.exists():
        if path.read_bytes() != data:
            raise FileExistsError("Refusing to overwrite changed file: " + str(path))
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        stream.write(data)


def verify_local_snapshot() -> None:
    manifest = json.loads((BASE / "upstream-manifest.json").read_text(encoding="utf-8"))
    if manifest["commit"] != COMMIT or manifest["git_tree_sha1"] != EXPECTED_TREE:
        raise ValueError("Local manifest does not match the pinned source")
    entries = manifest["files"]
    expected_paths = {entry["path"] for entry in entries}
    actual_paths = {p.relative_to(UPSTREAM).as_posix() for p in UPSTREAM.rglob("*") if p.is_file()}
    if actual_paths != expected_paths or len(entries) != len(expected_paths):
        raise ValueError("Local source file set differs from the manifest")
    for entry in entries:
        path = UPSTREAM / entry["path"]
        if not path.resolve().is_relative_to(UPSTREAM.resolve()) or path.is_symlink():
            raise ValueError("Invalid local source path: " + entry["path"])
        data = path.read_bytes()
        if (len(data) != entry["bytes"]
                or hashlib.sha256(data).hexdigest() != entry["sha256"]
                or git_hash("blob", data).hex() != entry["git_blob_sha1"]):
            raise ValueError("Modified source: " + entry["path"])
    if calculate_tree(entries) != EXPECTED_TREE:
        raise ValueError("Local source Git tree does not match the pinned commit")
    if ARCHIVE.exists() and hashlib.sha256(ARCHIVE.read_bytes()).hexdigest() != manifest["archive_sha256"]:
        raise ValueError("Local archive checksum mismatch")
    print(json.dumps({"commit": COMMIT, "files_verified": len(entries),
                      "git_tree_sha1": EXPECTED_TREE, "archive_present": ARCHIVE.exists(),
                      "network_used": False, "verification": "PASS"}, indent=2))


def main() -> None:
    if (BASE / "upstream-manifest.json").exists():
        verify_local_snapshot()
        return
    payload = load_archive()
    members = unpack_verified(payload)
    expected_paths = {record["path"] for record, _ in members}
    if UPSTREAM.exists():
        actual_paths = {p.relative_to(UPSTREAM).as_posix() for p in UPSTREAM.rglob("*") if p.is_file()}
        unexpected = actual_paths - expected_paths
        if unexpected:
            raise ValueError("Unexpected files in upstream: " + ", ".join(sorted(unexpected)))
    # Preflight every existing source before writing anything.
    for record, data in members:
        path = UPSTREAM / record["path"]
        if path.exists() and path.read_bytes() != data:
            raise FileExistsError("Modified upstream file: " + str(path))
    for required in ("LICENSE", "THIRD_PARTY_NOTICES.md", ".project", ".cproject", "code/image.c"):
        if required not in expected_paths:
            raise ValueError("Missing required source: " + required)
    write_once_or_verify(ARCHIVE, payload)
    for record, data in members:
        write_once_or_verify(UPSTREAM / record["path"], data)
    manifest = {
        "retrieved_date": "2026-11-13",
        "repository": "https://github.com/" + REPOSITORY,
        "branch_at_selection": "main",
        "commit": COMMIT,
        "git_tree_sha1": EXPECTED_TREE,
        "archive_url": SOURCE_URL,
        "archive_path": ARCHIVE.relative_to(ROOT).as_posix(),
        "archive_bytes": len(payload),
        "archive_sha256": hashlib.sha256(payload).hexdigest(),
        "archive_retained": True,
        "file_count": len(members),
        "source_bytes": sum(record["bytes"] for record, _ in members),
        "verification": "ZIP CRC + every Git blob + complete Git root tree matches pinned commit",
        "execution_status": "source snapshot only; not compiled, flashed, or executed",
        "files": sorted((record for record, _ in members), key=lambda record: record["path"]),
    }
    manifest_bytes = (json.dumps(manifest, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
    write_once_or_verify(BASE / "upstream-manifest.json", manifest_bytes)
    print(json.dumps({key: manifest[key] for key in ("commit", "git_tree_sha1", "archive_bytes", "file_count", "source_bytes", "verification")}, indent=2))


if __name__ == "__main__":
    main()
