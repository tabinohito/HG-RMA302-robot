"""Persist fault frames before an ESC stop/retry clears their details."""

import json
from datetime import datetime, timezone
from pathlib import Path


FAULT_LOG = Path("artifacts/diagnostics/esc-faults.jsonl")


def write_fault_events(events, path=FAULT_LOG):
    if not events:
        return
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as file:
        for event in events:
            file.write(json.dumps({"logged_at": datetime.now(timezone.utc).isoformat(), **event},
                                  ensure_ascii=False) + "\n")
