"""Mission command definitions and serialization helpers."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict

MISSION_VERSION = 1


@dataclass(frozen=True)
class MissionCommand:
    mission_type: str
    params: Dict[str, Any]
    timestamp: float = field(default_factory=lambda: time.time())
    version: int = MISSION_VERSION

    def to_payload(self) -> Dict[str, Any]:
        return {
            "type": self.mission_type,
            "params": self.params,
            "timestamp": self.timestamp,
            "version": self.version,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_payload())

    @staticmethod
    def from_payload(payload: Dict[str, Any]) -> "MissionCommand":
        return MissionCommand(
            mission_type=str(payload.get("type", "")),
            params=dict(payload.get("params", {})),
            timestamp=float(payload.get("timestamp", time.time())),
            version=int(payload.get("version", MISSION_VERSION)),
        )

    @staticmethod
    def from_json(raw: str) -> "MissionCommand":
        data = json.loads(raw)
        if not isinstance(data, dict):
            raise ValueError("Mission payload must be a JSON object")
        return MissionCommand.from_payload(data)


MISSION_FORM_UP_CIRCLE = "FORM_UP_CIRCLE"
MISSION_HOLD_POSITION = "HOLD_POSITION"
MISSION_GOTO_LATLON = "GOTO_LATLON"
