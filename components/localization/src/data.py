"""This module provides some basic data types used by the ranging and localization."""

from logging import warning
from json import JSONDecodeError, JSONDecoder, JSONEncoder
from typing import List, NamedTuple, Optional, Dict


RX = 1
TX = 2


class ActiveMeasurement(NamedTuple):
    """Data structure to store an active measurement."""

    a: int
    """ID of the node receiving the last message in TWR."""
    b: int
    """ID of the other node that participates in ranging."""
    distance: float
    """"The distance between `a` and `b` as measured by `a`"""
    time: float
    """The unix time at which this measurement was done."""

    def __eq__(self, other):
        return self.distance == other.distance

    def __lt__(self, other):
        return self.distance < other.distance


class PassiveMeasurement(NamedTuple):
    """Data structure to store an passive measurement."""

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
    """A structure containing information about the time of sending or receiving a certain message.

    Depending of the context this can either represent the time when node with ID ``addr`` sent a message with sequence number ``sn`` or the time at which that message was received.
    """

    addr: int
    """The sender or receiver ID, respectively."""
    sn: int
    "The sequence number of the message."
    ts: int
    """The reception or transmission time, respectively."""

    def __eq__(self, other):
        # Is this definition of equality a good idea? I think not.
        return self.addr == other.addr and self.sn == other.sn

    def __lt__(self, other):
        return self.sn < other.sn


class Message(NamedTuple):
    """A data structure for messages as they are exchanged by the UWB boards."""

    type: int
    """Whether this is a received or transmitted message.
    
    See constants ``RX`` and ``TX`` from this module.
    """
    clock_offset_ratio: Optional[float]
    """A value which represents the difference in clock speeds of both modules.
    
    The ``clock_offset_ratio`` can be obtained by comparing the frequency of the incoming signal to the own transmission frequency. See *Carrier Frequency Offset*"""
    tx: TimingInfo
    """Information about the transmission."""
    rx: List[TimingInfo]
    """Information about previously received messages."""


def parse_json_message(json_str: str) -> Optional[Message]:
    """Convert a string containing a ranging messages to the ``Message`` data type.

    Args:
        json_str: A string (hopefully) containing a message.

    Returns:
        The message if it could be parsed or ``None`` otherwise.
    """

    def parse_timing_info(json_dict: Dict) -> TimingInfo:
        return TimingInfo(
            addr=int(json_dict["addr"], 16), sn=json_dict["sn"], ts=json_dict["ts"]
        )

    try:
        json_dict = JSONDecoder().decode(json_str)
        if json_dict:
            clock_offset_ratio = None
            if "clock_ratio_offset" in json_dict:
                clock_offset_ratio = float(json_dict["clock_ratio_offset"])
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


def jsonify_message(message: Message) -> str:
    """Convert a ``Message`` to a JSON string.

    This is the reverse function to ``parse_json_message``.

    Args:
        message: The ranging mesesage to be encoded.

    Returns:
        The jsonified message.+
    """

    def jsonify_timing_info(t):
        return {"addr": hex(t.addr), "sn": t.sn, "ts": t.ts}

    message_dict = {
        "type": "rx" if message.type == RX else "tx",
        "tx": jsonify_timing_info(message.tx),
        "rx": list(map(jsonify_timing_info, message.rx)),
    }
    if message.clock_offset_ratio:
        message_dict["clock_offset_ratio"] = message.clock_offset_ratio
    return JSONEncoder().encode(message_dict)
