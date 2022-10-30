from collections import deque
from data import ActiveMeasurement, TimingInfo, RX, TX, Message
from ranging import RangingNode


MESSAGES = [
    Message(
        type=TX,
        clock_offset_ratio=None,
        tx=TimingInfo(addr=0xBBBB, sn=2, ts=600_000),
        rx=[TimingInfo(addr=0xAAAA, sn=2, ts=500_000)],
    ),
    Message(
        type=RX,
        clock_offset_ratio=None,
        tx=TimingInfo(addr=0xAAAA, sn=2, ts=400_000),
        rx=[TimingInfo(addr=0xBBBB, sn=1, ts=300_000)],
    ),
    Message(
        type=TX,
        clock_offset_ratio=None,
        tx=TimingInfo(addr=0xBBBB, sn=1, ts=200_000),
        rx=[TimingInfo(addr=0xAAAA, sn=1, ts=100_000)],
    ),
    Message(
        type=RX,
        clock_offset_ratio=None,
        tx=TimingInfo(addr=0xAAAA, sn=1, ts=0),
        rx=[],
    ),
]


class MockRangingNode(RangingNode):
    def __init__(self, measurement_callback):
        super().__init__( measurement_cb=measurement_callback)

    def run(self):
        pass

    def send_data(self, data):
        pass


def test_ranging():
    def cb(measurements):
        print(measurements)
        assert (
            measurements
            and abs(
                [
                    m
                    for m in measurements
                    if isinstance(m, ActiveMeasurement) and m.a == 0xBBBB
                ][0].distance
                - 469.175
            )
            < 1
        )

    ranging_node = MockRangingNode(cb)
    ranging_node.msg_storage = deque(MESSAGES[1:])
    print(ranging_node.msg_storage)
    ranging_node.handle_message(MESSAGES[0])


if __name__ == "__main__":
    test_ranging()
