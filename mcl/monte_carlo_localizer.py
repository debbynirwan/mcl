"""Monte Carlo Localizer
Description:
    This ros2 node tries to localize the robot in the given map.
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
import rclpy
import yaml
import os
import numpy as np

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from geometry_msgs.msg import Quaternion, Pose, Point, PoseStamped, PoseArray
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from ament_index_python.packages import get_package_share_directory
from mcl import motion_model, util
from math import degrees
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from mcl.sensor_model import Map, LikelihoodFields, BeamModel
from typing import List

MILLISECONDS = 0.001
ALMOST_ZERO = 1e-15


class Particle:
    def __init__(self, pose: Pose, weight: float):
        self.pose: Pose = pose
        self.weight: float = weight


class MonteCarloLocalizer(Node):

    def __init__(self):
        super().__init__('monte_carlo_localizer')
        self.create_subscription(Odometry, '/odom',
                                 self.odometry_callback, 1)
        self.create_subscription(LaserScan, '/scan',
                                 self.scan_callback, 1)
        self._mcl_path = Path()
        self._odom_path = Path()
        self._mcl_path_pub = self.create_publisher(Path, '/mcl_path', 10)
        self._odom_path_pub = self.create_publisher(Path, '/odom_path', 10)
        self._particle_pub = self.create_publisher(PoseArray, '/particlecloud', 10)
        self.create_timer(100 * MILLISECONDS, self.timer_callback)
        self._particles: List[Particle] = []

        self._last_used_odom: Pose = None
        self._last_odom: Pose = None
        self._current_pose: Pose = None
        self._motion_model_cfg = None
        self._mcl_cfg = None
        self._last_scan: LaserScan = None
        self._updating = False

        package_dir = get_package_share_directory('mcl')
        motion_model_cfg_file = 'resource/motion_model.yaml'
        with open(os.path.join(package_dir, motion_model_cfg_file)) as cfg_file:
            self._motion_model_cfg = yaml.load(cfg_file, Loader=yaml.FullLoader)
        if self._motion_model_cfg is None:
            raise RuntimeError(f"{motion_model_cfg_file} not found")

        mcl_cfg_file = 'resource/mcl.yaml'
        with open(os.path.join(package_dir, mcl_cfg_file)) as cfg_file:
            self._mcl_cfg = yaml.load(cfg_file, Loader=yaml.FullLoader)
        if self._mcl_cfg is None:
            raise RuntimeError(f"{mcl_cfg_file} not found")

        self._map = Map('resource/epuck_world_map.yaml', self.get_logger())
        if self._mcl_cfg['likelihood_model']:
            self._sensor_model = LikelihoodFields(self._map)
        else:
            self._sensor_model = BeamModel(self._map)
        self._map_publisher = self.create_publisher(
            OccupancyGrid,
            '/map',
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            )
        )

        self._initialize_pose()
        self._initialize_particles_gaussian()

        self._tf_publisher = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'odom'
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        self._tf_publisher.sendTransform(tf)
        self._publish_map()

    def timer_callback(self):
        if self._last_used_odom and self._last_odom:
            if self._mcl_cfg['adaptive']:
                self._update_adaptive()
            else:
                self._update()
            self._publish_particles()
            self._last_used_odom = self._last_odom

    def odometry_callback(self, msg: Odometry):
        if not self._updating:
            self._last_odom = msg.pose.pose

    def scan_callback(self, msg: LaserScan):
        if not self._updating:
            self._last_scan = msg

    def _initialize_pose(self):
        position = Point(x=0.0,
                         y=0.0,
                         z=0.0)
        orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)
        self._current_pose = Pose(position=position,
                                  orientation=orientation)
        self._last_used_odom = self._current_pose
        self._last_odom = self._current_pose

    def _initialize_particles_gaussian(self, pose: Pose = None, scale: float = 0.05):
        if pose is None:
            pose = self._current_pose

        x_list = list(np.random.normal(loc=pose.position.x, scale=scale, size=self._mcl_cfg['num_of_particles'] - 1))
        y_list = list(np.random.normal(loc=pose.position.y, scale=scale, size=self._mcl_cfg['num_of_particles'] - 1))
        current_yaw = util.yaw_from_quaternion(pose.orientation)
        yaw_list = list(np.random.normal(loc=current_yaw, scale=0.01, size=self._mcl_cfg['num_of_particles'] - 1))

        initial_weight = 1.0 / float(self._mcl_cfg['num_of_particles'])

        for x, y, yaw in zip(x_list, y_list, yaw_list):
            position = Point(x=x, y=y, z=0.0)
            orientation = util.euler_to_quaternion(yaw, 0.0, 0.0)
            temp_pose = Pose(position=position, orientation=orientation)
            self._particles.append(Particle(temp_pose, initial_weight))

        self._particles.append(Particle(pose, initial_weight))

    def _normalize_particles(self, particles: List[Particle], log):
        sum_of_weights = 0.0
        for particle in particles:
            sum_of_weights += particle.weight
        if sum_of_weights < ALMOST_ZERO:
            return [Particle(particle.pose, 1.0 / len(particles)) for particle in particles]
        else:
            return [Particle(particle.pose, particle.weight / sum_of_weights) for particle in particles]

    def _resample(self, particles: List[Particle]):
        weights = [particle.weight for particle in particles]
        cum_sums = np.cumsum(weights).tolist()
        n = 0
        new_samples = []

        initial_weight = 1.0 / float(self._mcl_cfg['num_of_particles'])
        while n < self._mcl_cfg['num_of_particles']:
            u = np.random.uniform(1e-6, 1, 1)[0]
            m = 0
            while cum_sums[m] < u:
                m += 1
            new_samples.append(Particle(particles[m].pose, initial_weight))
            n += 1

        return new_samples

    @staticmethod
    def _best_particle(particles: List[Particle]):
        weight = 0.0
        pose = None
        for particle in particles:
            if particle.weight > weight:
                weight = particle.weight
                pose = particle.pose
        return pose, weight

    @staticmethod
    def _pose_estimate(particles: List[Particle]):
        x = 0.0
        y = 0.0
        yaw = 0.0
        for particle in particles:
            x += particle.pose.position.x * particle.weight
            y += particle.pose.position.y * particle.weight
            yaw += util.yaw_from_quaternion(particle.pose.orientation) * particle.weight
        position = Point(x=x, y=y, z=0.0)
        orientation = util.euler_to_quaternion(yaw=yaw, pitch=0.0, roll=0.0)
        return Pose(position=position, orientation=orientation)

    @staticmethod
    def _should_resample(particles: List[Particle], resampling_threshold):
        sum_weights_squared = 0
        for particle in particles:
            sum_weights_squared += particle.weight ** 2

        return 1.0 / sum_weights_squared < resampling_threshold

    @staticmethod
    def _generate_sample_index(particles: List[Particle]):
        weights = [particle.weight for particle in particles]
        cum_sums = np.cumsum(weights).tolist()
        m = 0
        u = np.random.uniform(1e-6, 1, 1)[0]
        while cum_sums[m] < u:
            m += 1

        return particles[m].pose

    @staticmethod
    def _compute_required_number_of_particles_kld(k, epsilon, upper_quantile):
        """
        Compute the number of samples needed within a particle filter when k bins in the multidimensional histogram contain
        samples. Use Wilson-Hilferty transformation to approximate the quantiles of the chi-squared distribution as proposed
        by Fox (2003).

        :param epsilon: Maxmimum allowed distance (error) between true and estimated distribution.
        :param upper_quantile: Upper standard normal distribution quantile for (1-delta) where delta is the probability that
        the error on the estimated distribution will be less than epsilon.
        :param k: Number of bins containing samples.
        :return: Number of required particles.
        """
        # Helper variable (part between curly brackets in (7) in Fox paper
        x = 1.0 - 2.0 / (9.0 * (k - 1)) + np.sqrt(2.0 / (9.0 * (k - 1))) * upper_quantile
        return np.ceil((k - 1) / (2.0 * epsilon) * x * x * x)

    def _publish_particles(self):
        if len(self._particles) == 0:
            return
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for particle in self._particles:
            msg.poses.append(particle.pose)
        self._particle_pub.publish(msg)

    def _publish_map(self):
        map = [-1] * self._map.width * self._map.height
        idx = 0
        for cell in self._map.data:
            map[idx] = int(cell * 100.0)
            idx += 1
        stamp = self.get_clock().now().to_msg()
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.info.resolution = self._map.resolution
        msg.info.width = self._map.width
        msg.info.height = self._map.width
        msg.info.origin.position.x = self._map.origin[0]
        msg.info.origin.position.y = self._map.origin[1]
        msg.data = map
        self._map_publisher.publish(msg)

    def _publish_mcl_path(self, published_pose: Pose):
        stamp = self.get_clock().now().to_msg()
        self._mcl_path.header.frame_id = 'map'
        self._mcl_path.header.stamp = stamp
        pose = PoseStamped()
        pose.header = self._mcl_path.header
        pose.pose = published_pose
        self._mcl_path.poses.append(pose)
        self._mcl_path_pub.publish(self._mcl_path)

    def _publish_odom_path(self, odom_pose: Pose):
        stamp = self.get_clock().now().to_msg()
        self._odom_path.header.frame_id = 'odom'
        self._odom_path.header.stamp = stamp
        pose = PoseStamped()
        pose.header = self._odom_path.header
        pose.pose = odom_pose
        self._odom_path.poses.append(pose)
        self._odom_path_pub.publish(self._odom_path)

    def _update_adaptive(self):
        if self._last_odom == self._last_used_odom:
            return

        self._updating = True
        new_particles = []
        bins_with_support = []
        number_of_new_particles = 0
        number_of_bins_with_support = 0
        number_of_required_particles = self._mcl_cfg['min_number_of_particles']

        while number_of_new_particles < number_of_required_particles:

            current_pose = self._generate_sample_index(self._particles)
            predicted_pose = motion_model.sample_odom_motion_model(current_pose,
                                                                   self._last_odom,
                                                                   self._last_used_odom,
                                                                   self._motion_model_cfg)
            importance_weight = self._sensor_model.update(self._last_scan, predicted_pose)

            new_particle = Particle(predicted_pose, importance_weight)
            new_particles.append(new_particle)
            number_of_new_particles += 1

            x = int(new_particle.pose.position.x / self._map.resolution)
            y = int(new_particle.pose.position.y / self._map.resolution)
            yaw = int(degrees(util.yaw_from_quaternion(new_particle.pose.orientation)))

            indices = [x, y, yaw]
            if indices not in bins_with_support:
                bins_with_support.append(indices)
                number_of_bins_with_support += 1

            if number_of_bins_with_support > 1:
                number_of_required_particles = self._compute_required_number_of_particles_kld(number_of_bins_with_support,
                                                                                              self._mcl_cfg['epsilon'],
                                                                                              self._mcl_cfg['upper_quantile'])
            number_of_required_particles = max(number_of_required_particles, self._mcl_cfg['min_number_of_particles'])
            number_of_required_particles = min(number_of_required_particles, self._mcl_cfg['max_number_of_particles'])

        self._particles.clear()
        self._particles = self._normalize_particles(new_particles, self.get_logger())
        self._current_pose = self._pose_estimate(self._particles)
        self._publish_mcl_path(self._current_pose)
        self._publish_odom_path(self._last_odom)
        self._updating = False

    def _update(self):
        if self._last_odom == self._last_used_odom:
            return

        self._updating = True
        new_particles = []

        for particle in self._particles:
            current_pose = particle.pose
            predicted_pose = motion_model.sample_odom_motion_model(current_pose,
                                                                   self._last_odom,
                                                                   self._last_used_odom,
                                                                   self._motion_model_cfg)
            prob = self._sensor_model.update(self._last_scan, predicted_pose)
            new_particles.append(Particle(predicted_pose, prob))

        self._particles.clear()
        self._particles = self._normalize_particles(new_particles, self.get_logger())
        self._current_pose = self._pose_estimate(self._particles)
        self._publish_mcl_path(self._current_pose)
        self._publish_odom_path(self._last_odom)

        if self._should_resample(self._particles, self._mcl_cfg['num_of_particles'] / 5.0):
            self._particles = self._resample(self._particles)

        self._updating = False


def main(args=None):
    rclpy.init(args=args)
    monte_carlo_localizer = MonteCarloLocalizer()
    rclpy.spin(monte_carlo_localizer)
    monte_carlo_localizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
