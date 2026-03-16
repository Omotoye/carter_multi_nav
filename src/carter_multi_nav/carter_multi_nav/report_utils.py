import csv
import json
import os
from typing import Iterable


def ensure_output_dir(output_dir: str) -> str:
    text = str(output_dir or "").strip()
    if not text:
        return ""
    absolute = os.path.abspath(os.path.expanduser(text))
    os.makedirs(absolute, exist_ok=True)
    return absolute


def write_json(path: str, payload):
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)

    temp_path = f"{path}.tmp"
    with open(temp_path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")
    os.replace(temp_path, path)


def append_csv(path: str, fieldnames: Iterable[str], row: dict):
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)

    fieldnames = list(fieldnames)
    write_header = not os.path.exists(path)
    with open(path, "a", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        if write_header:
            writer.writeheader()
        writer.writerow({key: row.get(key, "") for key in fieldnames})

