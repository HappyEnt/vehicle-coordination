import json
from logging import error, warning
from json import JSONDecodeError, JSONDecoder
from typing import List, NamedTuple, Optional, Dict


RX = 1
TX = 2

class ActiveMeasurement(NamedTuple):
    a: int
    b: int
    """"The distance between `a` and `b` as measured by `a`"""
    distance: float
    time: float

    def __eq__(self, other):
        return self.distance == other.distance

    def __lt__(self, other):
        return self.distance < other.distance


class PassiveMeasurement(NamedTuple):
    a: int
    b: int
    c: int
    distance: float
    time: float

    def __eq__(self, other):
        return self.distance == other.distance

    def __lt__(self, other):
        return self.distance < other.distance


class TimingInfo(NamedTuple):
    addr: int
    sn: int
    ts: int

    def __eq__(self, other):
        # Is this definition of equality a good idea? I think not.
        return self.addr == other.addr and self.sn == other.sn

    def __lt__(self, other):
        return self.sn < other.sn


class Message(NamedTuple):
    type: int
    clock_offset_ratio: Optional[float]
    tx: TimingInfo
    rx: List[TimingInfo]


def parse_json_message(json_str: str) -> Optional[Message]:
    def parse_timing_info(json_dict: Dict) -> TimingInfo:
        return TimingInfo(
            addr=int(json_dict["addr"], 16), sn=json_dict["sn"], ts=json_dict["ts"]
        )

    try:
        json_dict = JSONDecoder().decode(json_str)
        if json_dict:
            clock_offset_ratio = None
            if "clock_ratio_offset" in json_dict:
                clock_offset_ratio=float(json_dict["clock_ratio_offset"])
            return Message(
                type=RX if json_dict["type"] == "rx" else TX,
                clock_offset_ratio=clock_offset_ratio,
                tx=parse_timing_info(json_dict["tx"]),
                rx=list(map(parse_timing_info, json_dict["rx"])),
            )
        else:
            warning("Decode Error")
            return None
    except (KeyError, JSONDecodeError):
        warning("Decode Error")
        return None
