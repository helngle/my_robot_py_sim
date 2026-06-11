#!/usr/bin/env python3

import copy
import math
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from PIL import Image
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile


class StaticMapPublisher(Node):
    def __init__(self) -> None:
        super().__init__("static_map_publisher")

        self.declare_parameter("map_yaml", "/home/zfb/semantic_map_ws/maps/Test052601/Test052601.yaml")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("costmap_topic", "/costmap")
        self.declare_parameter("inflation_layer1", 0.5)
        self.declare_parameter("inflation_layer2", 0.6)
        self.declare_parameter("publish_period_sec", 1.0)

        self.map_yaml = str(self.get_parameter("map_yaml").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.costmap_topic = str(self.get_parameter("costmap_topic").value)
        self.inflation_layer1 = float(self.get_parameter("inflation_layer1").value)
        self.inflation_layer2 = float(self.get_parameter("inflation_layer2").value)
        self.publish_period_sec = float(self.get_parameter("publish_period_sec").value)

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.map_publisher = self.create_publisher(OccupancyGrid, self.map_topic, qos)
        self.costmap_publisher = self.create_publisher(OccupancyGrid, self.costmap_topic, qos)

        self.map_msg = self._load_map(self.map_yaml)
        self.costmap_msg = self._inflate_map(
            self.map_msg,
            layer1_dist=self.inflation_layer1,
            layer2_dist=self.inflation_layer2,
        )

        self._publish_maps()
        self.timer = (
            self.create_timer(self.publish_period_sec, self._publish_maps)
            if self.publish_period_sec > 0.0
            else None
        )

        self.get_logger().info(
            f"Publishing map {self.map_msg.info.width}x{self.map_msg.info.height} on {self.map_topic} "
            f"and costmap on {self.costmap_topic}"
        )

    def _publish_maps(self) -> None:
        now = self.get_clock().now().to_msg()
        self.map_msg.header.stamp = now
        self.costmap_msg.header.stamp = now
        self.map_publisher.publish(self.map_msg)
        self.costmap_publisher.publish(self.costmap_msg)

    def _load_map(self, yaml_path: str) -> OccupancyGrid:
        yaml_file = Path(yaml_path).expanduser()
        if not yaml_file.is_file():
            raise FileNotFoundError(f"Map yaml not found: {yaml_file}")

        yaml_text = yaml_file.read_text(encoding="utf-8")
        if yaml_text.startswith("%YAML:"):
            yaml_text = "\n".join(yaml_text.splitlines()[1:])
        map_info = yaml.safe_load(yaml_text)
        if not isinstance(map_info, dict):
            raise ValueError(f"Invalid map yaml: {yaml_file}")

        image_path = Path(str(map_info["image"]))
        if not image_path.is_absolute():
            image_path = yaml_file.parent / image_path
        if not image_path.is_file():
            raise FileNotFoundError(f"Map image not found: {image_path}")

        pixels = np.asarray(Image.open(image_path).convert("L"), dtype=np.uint8)
        height, width = pixels.shape

        negate = int(map_info.get("negate", 0))
        occupied_thresh = float(map_info.get("occupied_thresh", 0.65))
        free_thresh = float(map_info.get("free_thresh", 0.196))

        values = pixels.astype(np.float32) / 255.0
        occupancy = values if negate else 1.0 - values

        data = np.full((height, width), -1, dtype=np.int8)
        data[occupancy > occupied_thresh] = 100
        data[occupancy < free_thresh] = 0

        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.info.resolution = float(map_info["resolution"])
        msg.info.width = int(width)
        msg.info.height = int(height)

        origin = map_info.get("origin", [0.0, 0.0, 0.0])
        yaw = float(origin[2]) if len(origin) > 2 else 0.0
        msg.info.origin.position.x = float(origin[0])
        msg.info.origin.position.y = float(origin[1])
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.z = math.sin(yaw / 2.0)
        msg.info.origin.orientation.w = math.cos(yaw / 2.0)

        msg.data = np.flipud(data).reshape(-1).astype(int).tolist()
        return msg

    def _inflate_map(
        self,
        original_map: OccupancyGrid,
        layer1_dist: float,
        layer2_dist: float,
    ) -> OccupancyGrid:
        width = original_map.info.width
        height = original_map.info.height
        resolution = original_map.info.resolution
        source = np.array(original_map.data, dtype=np.int16).reshape((height, width))
        obstacle_mask = source == 100

        inflated = np.zeros((height, width), dtype=np.int8)
        if obstacle_mask.any():
            try:
                from scipy.ndimage import distance_transform_edt

                distance_cells = distance_transform_edt(~obstacle_mask)
                distance_meters = distance_cells * resolution
                inflated[(distance_meters > 0) & (distance_meters <= layer2_dist)] = 98
                inflated[(distance_meters > 0) & (distance_meters <= layer1_dist)] = 100
            except ImportError:
                inflated = self._inflate_map_fallback(obstacle_mask, resolution, layer1_dist, layer2_dist)

        inflated[obstacle_mask] = 100

        costmap = OccupancyGrid()
        costmap.header = copy.deepcopy(original_map.header)
        costmap.info = copy.deepcopy(original_map.info)
        costmap.data = inflated.reshape(-1).astype(int).tolist()
        return costmap

    def _inflate_map_fallback(
        self,
        obstacle_mask: np.ndarray,
        resolution: float,
        layer1_dist: float,
        layer2_dist: float,
    ) -> np.ndarray:
        height, width = obstacle_mask.shape
        inflated = np.zeros((height, width), dtype=np.int8)
        layer1_sq = (layer1_dist / resolution) ** 2
        layer2_sq = (layer2_dist / resolution) ** 2
        radius = int(math.ceil(layer2_dist / resolution))
        ys, xs = np.where(obstacle_mask)

        for oy, ox in zip(ys, xs):
            for dy in range(-radius, radius + 1):
                ny = oy + dy
                if not 0 <= ny < height:
                    continue
                for dx in range(-radius, radius + 1):
                    nx = ox + dx
                    if not 0 <= nx < width:
                        continue
                    dist_sq = dx * dx + dy * dy
                    if dist_sq <= layer1_sq:
                        inflated[ny, nx] = 100
                    elif dist_sq <= layer2_sq and inflated[ny, nx] != 100:
                        inflated[ny, nx] = 98

        return inflated


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = StaticMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
