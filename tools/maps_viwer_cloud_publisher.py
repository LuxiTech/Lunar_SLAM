#!/usr/bin/env python3
"""Publish an RTAB-Map-exported RGB PLY as a ROS 2 PointCloud2 topic."""

import argparse
from pathlib import Path
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


SCALAR_FORMATS = {
    "char": "b", "int8": "b", "uchar": "B", "uint8": "B",
    "short": "h", "int16": "h", "ushort": "H", "uint16": "H",
    "int": "i", "int32": "i", "uint": "I", "uint32": "I",
    "float": "f", "float32": "f", "double": "d", "float64": "d",
}


def read_colored_ply(path: Path):
    """Read x/y/z/r/g/b vertices from an ASCII or binary_little_endian PLY."""
    with path.open("rb") as stream:
        if stream.readline().strip() != b"ply":
            raise ValueError("not a PLY file")
        file_format = None
        vertex_count = None
        properties = []
        in_vertex_element = False
        while True:
            raw_line = stream.readline()
            if not raw_line:
                raise ValueError("PLY header is incomplete")
            words = raw_line.decode("ascii").strip().split()
            if words == ["end_header"]:
                break
            if words[:1] == ["format"] and len(words) >= 2:
                file_format = words[1]
            elif words[:2] == ["element", "vertex"] and len(words) == 3:
                vertex_count = int(words[2])
                in_vertex_element = True
            elif words[:1] == ["element"]:
                in_vertex_element = False
            elif in_vertex_element and words[:1] == ["property"]:
                if len(words) != 3 or words[1] == "list" or words[1] not in SCALAR_FORMATS:
                    raise ValueError("unsupported PLY vertex property")
                properties.append((words[2], words[1]))

        if file_format not in {"ascii", "binary_little_endian"}:
            raise ValueError(f"unsupported PLY format: {file_format}")
        if vertex_count is None:
            raise ValueError("PLY has no vertex element")
        names = [name for name, _ in properties]
        if not {"x", "y", "z"}.issubset(names):
            raise ValueError("PLY vertices must contain x, y and z")

        if file_format == "ascii":
            vertices = [dict(zip(names, stream.readline().decode("ascii").split())) for _ in range(vertex_count)]
        else:
            format_string = "<" + "".join(SCALAR_FORMATS[kind] for _, kind in properties)
            unpack = struct.Struct(format_string).unpack
            record_size = struct.calcsize(format_string)
            vertices = []
            for _ in range(vertex_count):
                data = stream.read(record_size)
                if len(data) != record_size:
                    raise ValueError("PLY vertex data is truncated")
                vertices.append(dict(zip(names, unpack(data))))

    points = []
    for vertex in vertices:
        red = max(0, min(255, round(float(vertex.get("red", 255)))))
        green = max(0, min(255, round(float(vertex.get("green", 255)))))
        blue = max(0, min(255, round(float(vertex.get("blue", 255)))))
        rgb = (red << 16) | (green << 8) | blue
        rgb_float = struct.unpack("<f", struct.pack("<I", rgb))[0]
        points.append((float(vertex["x"]), float(vertex["y"]), float(vertex["z"]), rgb_float))
    return points


class CloudPublisher(Node):
    def __init__(self, path: Path, topic: str) -> None:
        super().__init__("maps_viwer_cloud_publisher")
        points = read_colored_ply(path)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(PointCloud2, topic, qos)
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        self.publisher.publish(point_cloud2.create_cloud(Header(frame_id="map"), fields, points))
        self.get_logger().info(f"Published {len(points)} colored points from {path}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("ply_path", type=Path)
    parser.add_argument("--topic", default="/maps_viwer/rtabmap_colored_cloud")
    args = parser.parse_args()
    if not args.ply_path.is_file():
        parser.error(f"PLY file does not exist: {args.ply_path}")
    rclpy.init()
    try:
        rclpy.spin(CloudPublisher(args.ply_path, args.topic))
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
