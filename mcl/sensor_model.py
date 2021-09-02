"""Sensor Model
Description:
    Sensor Models: Beam Model for Range Sensor and Likelihood Fields
License:
    Copyright 2021 Debby Nirwan
    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at
       http://www.apache.org/licenses/LICENSE-2.0
    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
"""
from typing import List
import cv2
import os
import yaml
import numpy as np
import sys

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from mcl import util
from math import pi, cos, sin, sqrt, isclose
from scipy.interpolate import interp1d
from scipy.stats import norm


ROBOT_DIAMETER = 0.07
ROBOT_RADIUS = ROBOT_DIAMETER / 2.0
INFRARED_MAX = 0.07
TOF_MAX = 2.00
MIN_READING = 0.0

INFRARED_READING = [
    0.000,
    0.005,
    0.010,
    0.015,
    0.020,
    0.030,
    0.040,
    0.050,
    0.060,
    0.070
]

INFRARED_STD_DEV = [
    0.0,
    0.003 * 0.005,
    0.007 * 0.010,
    0.0406 * 0.015,
    0.01472 * 0.020,
    0.0241 * 0.030,
    0.0287 * 0.040,
    0.04225 * 0.050,
    0.03065 * 0.060,
    0.04897 * 0.070
]

TOF_READING = [
    0.00,
    0.05,
    0.10,
    0.20,
    0.50,
    1.00,
    1.70,
    2.00
]

TOF_STD_DEV = [
    0.0,
    0.032 * 0.05,
    0.019 * 0.10,
    0.009 * 0.20,
    0.007 * 0.50,
    0.010 * 1.00,
    0.013 * 1.70,
    0.0
]


INFRARED_SENSOR_ANGLE = [
    -15 * pi / 180,   # ps0
    -45 * pi / 180,   # ps1
    -90 * pi / 180,   # ps2
    -150 * pi / 180,  # ps3
    150 * pi / 180,   # ps4
    90 * pi / 180,    # ps5
    45 * pi / 180,    # ps6
    15 * pi / 180,    # ps7
]


DISTANCE_SENSOR_TABLE = {
    0.0: ROBOT_RADIUS + 2.00,   # tof
    -15 * pi / 180: ROBOT_RADIUS + 0.070,  # ps0
    -45 * pi / 180: ROBOT_RADIUS + 0.070,  # ps1
    -90 * pi / 180: ROBOT_RADIUS + 0.070,  # ps2
    -150 * pi / 180: ROBOT_RADIUS + 0.070,  # ps3
    150 * pi / 180: ROBOT_RADIUS + 0.070,  # ps4
    90 * pi / 180: ROBOT_RADIUS + 0.070,  # ps5
    45 * pi / 180: ROBOT_RADIUS + 0.070,  # ps6
    15 * pi / 180: ROBOT_RADIUS + 0.070,  # ps7
}


class Map:
    def __init__(self, cfg_file, logger):
        package_dir = get_package_share_directory('mcl')
        self._map_cfg = None
        with open(os.path.join(package_dir, cfg_file)) as cfg:
            self._map_cfg = yaml.load(cfg, Loader=yaml.FullLoader)
        if self._map_cfg is None:
            raise RuntimeError(f"{cfg_file} not found")
        self.image_name = self._map_cfg['image']
        self.mode = self._map_cfg['mode']
        self.resolution = self._map_cfg['resolution']
        self.origin = self._map_cfg['origin']
        self.negate = self._map_cfg['negate']
        self.occupied_thresh = self._map_cfg['occupied_thresh']
        self.free_thresh = self._map_cfg['free_thresh']
        image_path = os.path.join(package_dir, 'resource/' + self.image_name)
        image_data = cv2.imread(image_path, -1)
        rotated = np.rot90(image_data)
        self.data = []
        for pixel in list(rotated.flatten()):
            self.data.append(1.0 - float(pixel) / 255.0)
        self.height = rotated.shape[0]
        self.width = rotated.shape[1]
        logger.debug(f"height:{self.height}, width:{self.width}, origin:{self.origin}")


class SimplePose:
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw


class LocationAndIndex:
    def __init__(self, x, y, index):
        self.x = x
        self.y = y
        self.index = index


class BeamModel:
    def __init__(self, map: Map):
        self._map = map
        self._infra_red_interpolation = interp1d(INFRARED_READING,
                                                 INFRARED_STD_DEV)
        self._tof_interpolation = interp1d(TOF_READING, TOF_STD_DEV)

    def update(self, scan: LaserScan, pose: Pose):
        if scan is None:
            return 1.0

        prob = 1.0
        pose_in_map = self._map_pose(pose)

        for reading in DISTANCE_SENSOR_TABLE.items():
            range_angle = pose_in_map.yaw + reading[0]
            range_x = pose_in_map.x + reading[1] * cos(range_angle)
            range_y = pose_in_map.y + reading[1] * sin(range_angle)

            if range_x >= self._map.width * self._map.resolution:
                range_x = (self._map.width - 1) * self._map.resolution
            if range_y >= self._map.height * self._map.resolution:
                range_y = (self._map.height - 1) * self._map.resolution

            expected = self._bresenham_line(int(pose_in_map.x / self._map.resolution),
                                            int(range_x / self._map.resolution),
                                            int(pose_in_map.y / self._map.resolution),
                                            int(range_y / self._map.resolution))
            real_reading = self._get_measurement_at(reading[0], scan)
            std_dev = self._get_std_dev(expected - ROBOT_RADIUS, reading[0] == 0.0)
            phit = self._get_prob(expected, real_reading, std_dev)
            prob *= phit

        return prob

    def _get_measurement_at(self, angle: float, scan: LaserScan) -> float:
        scan_angle = scan.angle_min
        for reading in scan.ranges:
            if isclose(scan_angle, angle, abs_tol=0.001):
                return reading
            scan_angle += scan.angle_increment

    def _map_pose(self, pose: Pose) -> SimplePose:
        x = pose.position.x
        y = pose.position.y
        yaw = util.yaw_from_quaternion(pose.orientation)
        map_x, map_y = util.location_in_map_meter(x, y, self._map)
        map_yaw = yaw
        return SimplePose(map_x, map_y, map_yaw)

    def _expected_reading(self, cells: list, max: float, pose: SimplePose) -> float:
        distance = ROBOT_RADIUS
        for cell in cells:
            if cell.index >= len(self._map.data):
                break
            if self._map.data[cell.index] >= self._map.occupied_thresh:
                delta_x = cell.x - int(pose.x / self._map.resolution)
                delta_y = cell.y - int(pose.y / self._map.resolution)
                distance = sqrt(delta_x ** 2 + delta_y ** 2) * self._map.resolution
                break
        return distance

    def _bresenham_line(self, x0, x1, y0, y1):
        # (https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm)
        distance = ROBOT_RADIUS
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        while True:
            index = y0 * self._map.width + x0
            if index >= len(self._map.data) or index < -len(self._map.data):
                continue
            if self._map.data[index] >= self._map.occupied_thresh:
                delta_x = x0 - x1
                delta_y = y0 - y1
                distance = sqrt(delta_x ** 2 + delta_y ** 2) * self._map.resolution
                break
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
        return distance

    def _get_std_dev(self, reading: float, tof: bool) -> tuple:
        if tof:
            if reading > TOF_MAX:
                reading = TOF_MAX
            elif reading < MIN_READING:
                reading = MIN_READING
            lookup_table = self._tof_interpolation
        else:
            if reading > INFRARED_MAX:
                reading = INFRARED_MAX
            elif reading < MIN_READING:
                reading = MIN_READING
            lookup_table = self._infra_red_interpolation

        return lookup_table(reading)

    def _get_prob(self, mean: float, reading: float, std_dev: float) -> tuple:
        if std_dev == 0.0:
            if isclose(reading, mean, abs_tol=0.001):
                return 1.0
            else:
                return 0.0
        probability = norm(loc=mean, scale=std_dev).pdf(reading)
        if probability >= 1.0:
            probability = 1.0

        return probability


class LikelihoodFields:
    def __init__(self, map: Map):
        self._map = map
        self._object_list: List[LocationAndIndex] = self._find_objects()
        package_dir = get_package_share_directory('mcl')
        self._sensor_cfg = None
        with open(os.path.join(package_dir, 'resource/sensor_model.yaml')) as cfg:
            self._sensor_cfg = yaml.load(cfg, Loader=yaml.FullLoader)
        if self._sensor_cfg is None:
            raise RuntimeError
        self._std_dev = self._sensor_cfg['likelihood_std_dev']

    def _find_objects(self):
        object_index_list = []
        for x in range(self._map.width):
            for y in range(self._map.height):
                index = y * self._map.width + x
                if index >= len(self._map.data) or index < -len(self._map.data):
                    continue
                if self._map.data[index] >= self._map.occupied_thresh:
                    object_index_list.append(LocationAndIndex(x, y, index))
        return object_index_list

    def _closest_object_dist(self, x, y):
        dist = sys.float_info.max
        for object in self._object_list:
            current_dist = sqrt(((object.x - x) ** 2) + ((object.y - y) ** 2))
            if current_dist < dist:
                dist = current_dist
        return dist

    def update(self, scan: LaserScan, pose: Pose):
        if scan is None:
            return 1.0

        prob = 1.0
        pose_in_map = self._map_pose(pose)

        sensor_angle = scan.angle_min
        for laser_range in scan.ranges:
            if laser_range < scan.range_max and laser_range > scan.range_min:
                range_angle = pose_in_map.yaw + sensor_angle
                range_x = pose_in_map.x + laser_range * cos(range_angle)
                range_y = pose_in_map.y + laser_range * sin(range_angle)
                if range_x >= self._map.width * self._map.resolution:
                    range_x = (self._map.width - 1) * self._map.resolution
                if range_y >= self._map.height * self._map.resolution:
                    range_y = (self._map.height - 1) * self._map.resolution
                dist = self._closest_object_dist(int(range_x / self._map.resolution),
                                                 int(range_y / self._map.resolution))
                phit = self._get_prob(dist * self._map.resolution)
                prob *= phit
            sensor_angle += scan.angle_increment

        return prob

    def _get_prob(self, dist: float) -> tuple:
        probability = norm(scale=self._std_dev).pdf(dist)
        if probability >= 1.0:
            probability = 1.0
        return probability

    def _map_pose(self, pose: Pose) -> SimplePose:
        x = pose.position.x
        y = pose.position.y
        yaw = util.yaw_from_quaternion(pose.orientation)
        map_x, map_y = util.location_in_map_meter(x, y, self._map)
        map_yaw = yaw
        return SimplePose(map_x, map_y, map_yaw)
