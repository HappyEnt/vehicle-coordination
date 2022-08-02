from argparse import ArgumentParser

from components.localization.src.localization import LocalizationNode
from components.localization.src.ranging import (
    SerialRangingNode,
)


def main():
    parser = ArgumentParser()
    parser.add_argument(
        "--port",
        "-p",
        metavar="P",
        type=str,
        required=False,
        help="Specify a serial port to which a UWB board is connected",
    )

    args = parser.parse_args()

    localization_node = LocalizationNode()
    ranging_nodes = []
    if args.port:
        ranging_nodes.append(
            SerialRangingNode(0, localization_node.handle_measurement, args.port)
        )

    for ranging_node in ranging_nodes:
        ranging_node.run()


if __name__ == "__main__":
    main()
