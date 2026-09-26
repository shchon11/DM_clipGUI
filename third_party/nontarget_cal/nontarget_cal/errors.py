"""Refusals: every reason the tool stops on purpose has a code and a message in English and Korean.
The CLI prints both, emits a `refusal` event and exits with status 2 (errors: status 1)."""
from __future__ import annotations


class Refusal(Exception):
    def __init__(self, code: str, msg: str, msg_ko: str, **details):
        super().__init__(f"[{code}] {msg}")
        self.code = code
        self.msg = msg
        self.msg_ko = msg_ko
        self.details = details

    def as_dict(self) -> dict:
        return {"code": self.code, "msg": self.msg, "msg_ko": self.msg_ko, **self.details}


# codes (the GUI can switch on them)
NO_BAG = "no_bag"
MISSING_TOPIC = "missing_topic"
NOT_ENOUGH_MOTION = "not_enough_motion"
NOT_ENOUGH_ROTATION = "not_enough_rotation"
NOT_ENOUGH_THERMAL = "not_enough_thermal_windows"
DISK = "insufficient_disk"
SYNC = "sync_broken"
EXPOSURE = "exposure_absurd"
AMBIGUOUS_NAMES = "ambiguous_camera_names"
UNKNOWN_NAMES = "unknown_camera_names"
BAG_DISAGREES = "bag_disagrees"
NO_GOOD_WINDOWS = "no_good_windows"
INIT = "bad_init"
