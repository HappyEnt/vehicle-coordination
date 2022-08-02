from logging import debug, info, warning
from typing import Dict, List, Optional, Tuple, Union

from scipy.constants import speed_of_light

from components.localization.src.config import TIME_UNIT
from components.localization.src.data import (
    ActiveMeasurement,
    Message,
    PassiveMeasurement,
    TimingInfo,
)


def perform_twr(
    message: Message,
    msg_storage: List[Message],
    tx_delays: Optional[Dict[int,float]] = None,
    rx_delays: Optional[Dict[int,float]] = None,
) -> List[Union[ActiveMeasurement, PassiveMeasurement]]:
    def find_message_sequence(
        messages: List[Message], endpoint: TimingInfo, active_addr
    ) -> Optional[Tuple[Message, Message, Message]]:
        a_msg = None
        a_sn = None
        b_msg = None
        b_sn = None
        c_msg = None
        for msg in messages:
            if a_sn and msg.tx.addr == endpoint.addr and msg.tx.sn == a_sn:
                try:
                    # ts = max([ts for ts in msg.rx if ts.addr == active_addr])
                    a_msg = Message(
                        tx=msg.tx,
                        # rx=[ts]
                        rx=[],
                    )
                    assert b_msg and c_msg
                    return (a_msg, b_msg, c_msg)
                except ValueError:
                    continue
            elif b_sn and msg.tx.addr == active_addr and msg.tx.sn == b_sn:
                try:
                    ts = max([ts for ts in msg.rx if ts.addr == endpoint.addr])
                    b_msg = Message(tx=msg.tx, rx=[ts])
                    a_sn = ts.sn
                except ValueError:
                    continue
            elif msg.tx.addr == endpoint.addr and msg.tx.sn == endpoint.sn:
                try:
                    ts = max([ts for ts in msg.rx if ts.addr == active_addr])
                    c_msg = Message(tx=msg.tx, rx=[ts])
                    b_sn = ts.sn
                except ValueError:
                    continue

        return None

    def get_ts(
        rx: List[TimingInfo], reference: TimingInfo
    ) -> int:
        for info in rx:
            if info == reference:
                return info.ts
        assert False
    measurements = []

    if not tx_delays:
        tx_delays = {}
    if not rx_delays:
        rx_delays = {}

    a_id = message.tx.addr
    if a_id not in tx_delays.keys():
        tx_delays[a_id] = 0
    if a_id not in rx_delays.keys():
        rx_delays[a_id] = 0

    for rx_timing_info in message.rx:
        b_id = rx_timing_info.addr

        if b_id not in tx_delays.keys():
            tx_delays[b_id] = 0
        if b_id not in rx_delays.keys():
            rx_delays[b_id] = 0

        # Trying to range with rx_timing_info.addr
        # We want to find the newest message we sent, containing a timestamp
        # of a message sent by our peer and that message
        sequence = find_message_sequence(msg_storage, rx_timing_info, message.tx.addr)
        if sequence:
            (a_msg, b_msg, c_msg) = sequence
            assert a_msg.tx in b_msg.rx and b_msg.tx in c_msg.rx

            # Time at A between sending b and receiving c
            r_a = get_ts(message.rx, c_msg.tx) - b_msg.tx.ts - tx_delays[a_id] - rx_delays[a_id]
            # Time at B between sending a and receiving b
            r_b = get_ts(c_msg.rx, b_msg.tx) - a_msg.tx.ts - tx_delays[b_id] - rx_delays[b_id]
            # Time at A between receiving a and sending b
            d_a = b_msg.tx.ts - get_ts(b_msg.rx, a_msg.tx) + tx_delays[a_id] + rx_delays[a_id]
            # Time at B between receiving b and sending c
            d_b = c_msg.tx.ts - get_ts(c_msg.rx, b_msg.tx) + tx_delays[b_id] + rx_delays[b_id]

            # TODO: Try to correct instead of filtering out
            if r_a < 0 or r_b < 0 or d_a < 0 or d_b < 0:
                continue

            tof = (r_a * r_b - d_a * d_b) / (r_a + r_b + d_a + d_b)
            distance = tof * TIME_UNIT * speed_of_light
            if distance < 0 or distance > 1_000_000:
                warning(f"Extremely unlikely distance of {distance}, R_A: {r_a}, R_B: {r_b}, D_A: {d_a}, D_B: {d_b}")
            debug(f"Distance between {message.tx.addr} and {rx_timing_info.addr}: {distance}")
            measurements.append(
                ActiveMeasurement(message.tx.addr, rx_timing_info.addr, distance)
            )

    return measurements
