from argparse import ArgumentParser
from logging import basicConfig, DEBUG, ERROR, INFO, WARNING
from threading import Thread
from time import sleep

from components.localization.src.localization import LocalizationNode
from components.localization.src.Localization_Tag import ParticleNode
from components.localization.src.ranging import DumpFileRangingNode, SerialRangingNode


def main():
    parser = ArgumentParser()
    parser.add_argument(
        "--port",
        "-p",
        metavar="PORT",
        type=str,
        required=False,
        help="Specify a serial port to which a UWB board is connected",
    )
    parser.add_argument(
        "--file",
        "-f",
        metavar="FILE",
        type=str,
        required=False,
        help="Specify a file containing recorded output from a UWB node",
    )
    parser.add_argument(
        "--log",
        "-l",
        metavar="LEVEL",
        type=str,
        default="INFO",
        help="The log level. One of: ERROR, WARNING, INFO, DEBUG",
    )

    args = parser.parse_args()

    if args.log == "ERROR":
        basicConfig(level=ERROR)
    elif args.log == "WARNING":
        basicConfig(level=WARNING)
    elif args.log == "INFO":
        basicConfig(level=INFO)
    elif args.log == "DEBUG":
        basicConfig(level=DEBUG)

    localization_node = ParticleNode()
    ranging_nodes = []
    if args.port:
        ranging_nodes.append(
            SerialRangingNode(0, localization_node.receive_measurements, args.port)
        )
    if args.file:
        ranging_nodes.append(
            DumpFileRangingNode(4, localization_node.receive_measurements, args.file)
        )

    localization_thread = Thread(target=localization_node.run)
    localization_thread.start()

    ranging_threads = [
        Thread(target=ranging_node.run) for ranging_node in ranging_nodes
    ]
    for thread in ranging_threads:
        thread.start()

    sleep(10)
    localization_node.illustrate_nodes_and_particles((100,0))


if __name__ == "__main__":
    main()
