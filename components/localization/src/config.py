"""This module provides some constants and default values."""

TIME_UNIT = 15.65 / 1_000_000_000_000

RX_DELAYS = {
    0: 77.8977994853012,
    256: 12.41623643762934,
    512: -23.796636682151608,
    768: -22.17242755695339,
    1024: 42.23565013797095,
    1280: -0.7720913969190917,
    1536: -13.277699237393,
    1792: -9.39894945994878,
}
TX_DELAYS = {
    0: -153.8229213593596,
    256: -118.5449861820107,
    512: -96.22487798246239,
    768: -18.916975464011536,
    1024: -107.17416474198698,
    1280: -86.80434511866139,
    1536: -38.294127139040036,
    1792: -1.6150401078513477,
}