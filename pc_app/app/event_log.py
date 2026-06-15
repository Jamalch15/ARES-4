from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from time import time
from typing import Any
from uuid import uuid4


@dataclass(frozen=True)
class EventRecord:
    id: str
    ts: float
    source: str
    message: str
    data: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "ts": self.ts,
            "source": self.source,
            "message": self.message,
            "data": self.data,
        }


class EventLog:
    def __init__(self, max_records: int = 300) -> None:
        self._records: deque[EventRecord] = deque(maxlen=max_records)

    def add(self, source: str, message: str, **data: Any) -> EventRecord:
        record = EventRecord(str(uuid4()), time(), source, message, data)
        self._records.append(record)
        return record

    def list(self, limit: int = 100) -> list[dict[str, Any]]:
        limit = max(1, min(int(limit), len(self._records) or 1))
        return [record.to_dict() for record in list(self._records)[-limit:]]

    def clear(self) -> None:
        self._records.clear()

