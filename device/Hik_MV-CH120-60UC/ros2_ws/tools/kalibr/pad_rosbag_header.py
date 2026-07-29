#!/usr/bin/env python3
"""Pad a ROS1 bag's file-header data area to the 4096-byte ROS1 convention."""

import argparse
import shutil
import struct


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source")
    parser.add_argument("destination")
    args = parser.parse_args()

    with open(args.source, "rb") as source, open(args.destination, "wb") as destination:
        magic = source.readline()
        if magic != b"#ROSBAG V2.0\n":
            raise RuntimeError("source is not a ROS1 V2 bag")

        header_size = struct.unpack("<I", source.read(4))[0]
        header = source.read(header_size)
        data_size = struct.unpack("<I", source.read(4))[0]
        data = source.read(data_size)
        if data_size > 4096:
            raise RuntimeError(f"file header data is unexpectedly large: {data_size}")

        destination.write(magic)
        destination.write(struct.pack("<I", header_size))
        destination.write(header)
        destination.write(struct.pack("<I", 4096))
        destination.write(data)
        destination.write(b" " * (4096 - data_size))
        shutil.copyfileobj(source, destination, length=16 * 1024 * 1024)


if __name__ == "__main__":
    main()
