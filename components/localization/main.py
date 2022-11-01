from argparse import ArgumentParser
import logging
import os
import sys
from threading import Thread

from serial import Serial

from filtering import GridParticleNode, ClassicParticleNode, ClassicAllAtOnce
from ranging import DumpFileRangingNode, SerialRangingNode


dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname + "../src/")


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
    parser.add_argument(
        "--localization",
        metavar="LOCAL",
        type=str,
        default="PF",
        help="The localization algorithm to be used. The default is PF (Particle Filter)."
        "Currently available are: PF (Particle Filter), GPF (Grid Particle Filter), "
        "CPF (Classic Particle Filter without simultaneous filtering.",
    )

    args = parser.parse_args()

    if args.log == "ERROR":
        logging.basicConfig(level=logging.ERROR)
    elif args.log == "WARNING":
        logging.basicConfig(level=logging.WARNING)
    elif args.log == "INFO":
        logging.basicConfig(level=logging.INFO)
    elif args.log == "DEBUG":
        logging.basicConfig(level=logging.DEBUG)

    if args.localization == "CPF":
        localization_node = ClassicParticleNode()
    elif args.localization == "GPF":
        localization_node = GridParticleNode()
    else:
        logging.warning(
            "Unrecognised argument for localization: %s. Falling back to default PF",
            args.localization,
        )
        localization_node = ClassicAllAtOnce()

    ranging_nodes = []
    if args.port:
        ranging_nodes.append(
            SerialRangingNode(
                localization_node.receive_measurements,
                Serial(args.port, baudrate=115200),
            )
        )
    if args.file:
        ranging_nodes.append(
            DumpFileRangingNode(localization_node.receive_measurements, args.file)
        )

    localization_thread = Thread(target=localization_node.run)
    localization_thread.start()

    ranging_threads = [
        Thread(target=ranging_node.run) for ranging_node in ranging_nodes
    ]
    for thread in ranging_threads:
        thread.start()


if __name__ == "__main__":
    main()
