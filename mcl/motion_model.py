"""Motion Model
Description:
    Odometry Motion Model
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
import numpy as np

from geometry_msgs.msg import Pose, Point
from math import atan2, sqrt, cos, sin
from mcl import util
from scipy.stats import norm

MOVED_TOO_CLOSE = 0.01


def sample_odom_motion_model(prev_true_xt: Pose,
                             latest_odom: Pose,
                             prev_odom: Pose,
                             cfg) -> float:
    if prev_odom == latest_odom:
        return prev_true_xt

    d1, dt, d2 = _calculate_pose_delta(latest_odom, prev_odom)

    alpha1 = cfg['alpha1']
    alpha2 = cfg['alpha2']
    alpha3 = cfg['alpha3']
    alpha4 = cfg['alpha4']

    std_dev_d1 = sqrt((alpha1 * (d1**2)) + (alpha2 * (dt**2)))
    std_dev_dt = sqrt((alpha3 * (dt**2)) + (alpha4 * (d1**2)) + (alpha4 * (d2**2)))
    std_dev_d2 = sqrt((alpha1 * (d2**2)) + (alpha2 * (dt**2)))

    noised1 = 0.0
    noisedt = 0.0
    noised2 = 0.0
    if std_dev_d1 > 0:
        noised1 = np.random.normal(scale=std_dev_d1)
    if std_dev_dt > 0:
        noisedt = np.random.normal(scale=std_dev_dt)
    if std_dev_d2 > 0:
        noised2 = np.random.normal(scale=std_dev_d2)

    t_d1 = util.angle_diff(d1, noised1)
    t_dt = dt + noisedt
    t_d2 = util.angle_diff(d2, noised2)

    curr_x = prev_true_xt.position.x
    curr_y = prev_true_xt.position.y
    curr_yaw = util.yaw_from_quaternion(prev_true_xt.orientation)

    x = curr_x + t_dt * cos(curr_yaw + t_d1)
    y = curr_y + t_dt * sin(curr_yaw + t_d1)
    yaw = curr_yaw + t_d1 + t_d2

    position = Point(x=x, y=y, z=0.0)
    orientation = util.euler_to_quaternion(yaw, 0, 0)

    return Pose(position=position, orientation=orientation)


def odom_motion_model(true_xt: Pose, prev_true_xt: Pose,
                      latest_odom: Pose, prev_odom: Pose, cfg) -> float:
    d1, dt, d2 = _calculate_pose_delta(latest_odom, prev_odom)
    t_d1, t_dt, t_d2 = _calculate_pose_delta(true_xt, prev_true_xt)

    alpha1 = cfg['alpha1']
    alpha2 = cfg['alpha2']
    alpha3 = cfg['alpha3']
    alpha4 = cfg['alpha4']
    p1 = norm(loc=d1 - t_d1, scale=sqrt((alpha1 * (t_d1**2)) + (alpha2 * (t_dt**2)))).pdf(d1 - t_d1)
    p2 = norm(loc=dt - t_dt, scale=sqrt((alpha3 * (t_dt**2)) + (alpha4 * (t_d1**2)) + (alpha4 * (t_d2**2)))).pdf(dt - t_dt)
    p3 = norm(loc=d2 - t_d2, scale=sqrt((alpha1 * (t_d2**2)) + (alpha2 * (t_dt**2)))).pdf(d2 - t_d2)

    return p1 * p2 * p3


def _calculate_pose_delta(xt: Pose, prev_xt: Pose):
    x = prev_xt.position.x
    y = prev_xt.position.y
    theta = util.yaw_from_quaternion(prev_xt.orientation)
    x_prime = xt.position.x
    y_prime = xt.position.y
    theta_prime = util.yaw_from_quaternion(xt.orientation)

    delta_translation = sqrt(((x - x_prime) ** 2) + ((y - y_prime) ** 2))
    delta_rotation1 = 0.0
    if delta_translation > MOVED_TOO_CLOSE:
        delta_rotation1 = util.angle_diff(atan2(y_prime - y, x_prime - x), theta)
    delta = util.angle_diff(theta_prime, theta)
    delta_rotation2 = util.angle_diff(delta, delta_rotation1)

    return delta_rotation1, delta_translation, delta_rotation2
