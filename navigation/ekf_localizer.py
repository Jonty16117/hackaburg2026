"""Extended Kalman Filter for continuous duck pose tracking.

3-state EKF: [x, y, theta] — duck pose relative to known fixed walls.
Walls are fixed constants (loaded from config) so no refinement needed.

Prediction: differential-drive odometry (motor speed commands).
Correction: sonar perpendicular distance matches predicted distance to wall.
"""

import math
import time
from collections import deque
from navigation.utils import normalize_angle
from navigation.config import (
    EKF_PROCESS_NOISE_XY,
    EKF_PROCESS_NOISE_THETA,
    EKF_SONAR_NOISE_CM2,
    EKF_INNOVATION_GATE_CM,
    EKF_LOST_COUNT_MAX,
    EKF_X_CORRECTION_INTERVAL_S,
)


class EKFLocalizer:
    def __init__(self, wall_map, initial_pose, wheel_base_cm=30.0,
                 max_speed_cm_s=100.0):
        self.wall_map = wall_map
        self.x = float(initial_pose[0])
        self.y = float(initial_pose[1])
        self.theta = float(initial_pose[2])
        self.wheel_base = wheel_base_cm
        self.max_speed = max_speed_cm_s

        self.cov = [
            [100.0, 0.0, 0.0],
            [0.0, 100.0, 0.0],
            [0.0, 0.0, 0.05],
        ]

        self.Q_xy = EKF_PROCESS_NOISE_XY
        self.Q_theta = EKF_PROCESS_NOISE_THETA
        self.R = EKF_SONAR_NOISE_CM2

        self.lost_count = 0
        self.total_corrections = 0
        self.x_corrections = 0
        self.last_x_correction_time = time.time()
        self.last_position = (float(initial_pose[0]), float(initial_pose[1]))
        self.last_move_time = time.time()
        self.recent_innovations = deque(maxlen=10)
        self._skip_correct = False

    def get_pose(self):
        return self.x, self.y, self.theta

    def set_pose(self, x, y, theta):
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)

    def needs_x_correction(self):
        return (time.time() - self.last_x_correction_time
                > EKF_X_CORRECTION_INTERVAL_S)

    def predict(self, left_speed, right_speed, dt):
        vl = left_speed * self.max_speed
        vr = right_speed * self.max_speed
        v = (vl + vr) / 2.0
        omega = (vr - vl) / self.wheel_base

        st = math.sin(self.theta)
        ct = math.cos(self.theta)

        self.x += v * ct * dt
        self.y += v * st * dt
        self.theta += omega * dt
        self.theta = normalize_angle(self.theta)

        F = [
            [1.0, 0.0, -v * st * dt],
            [0.0, 1.0, v * ct * dt],
            [0.0, 0.0, 1.0],
        ]

        self.cov = _mat_mul_3x3(F, _mat_mul_3x3(
            self.cov, _transpose_3x3(F)))

        dt2 = dt * dt
        self.cov[0][0] += self.Q_xy * dt2
        self.cov[1][1] += self.Q_xy * dt2
        self.cov[2][2] += self.Q_theta * dt2

        dx = self.x - self.last_position[0]
        dy = self.y - self.last_position[1]
        if abs(dx) > 2 or abs(dy) > 2:
            self.last_move_time = time.time()
            self.last_position = (self.x, self.y)

    def correct(self, sonar_cm):
        if sonar_cm is None:
            return

        wall_idx, predicted = self.wall_map.nearest_visible_wall(
            self.x, self.y, self.theta,
        )
        if wall_idx is None:
            return

        wall = self.wall_map.walls[wall_idx]
        signed = wall.signed_distance(self.x, self.y)
        sign = 1.0 if signed >= 0 else -1.0
        innovation = sonar_cm - predicted

        self.recent_innovations.append(innovation)

        if abs(innovation) > EKF_INNOVATION_GATE_CM:
            self.lost_count += 1
            return

        H = [sign * wall.A, sign * wall.B, 0.0]

        PHt = _mat_vec_mul(self.cov, H)
        S = H[0] * PHt[0] + H[1] * PHt[1] + H[2] * PHt[2] + self.R

        if S < 1e-12:
            return

        K = [PHt[0] / S, PHt[1] / S, PHt[2] / S]

        self.x += K[0] * innovation
        self.y += K[1] * innovation
        self.theta += K[2] * innovation
        self.theta = normalize_angle(self.theta)

        KH = [
            [K[0] * H[0], K[0] * H[1], K[0] * H[2]],
            [K[1] * H[0], K[1] * H[1], K[1] * H[2]],
            [K[2] * H[0], K[2] * H[1], K[2] * H[2]],
        ]
        I_KH = [
            [1.0 - KH[0][0], -KH[0][1], -KH[0][2]],
            [-KH[1][0], 1.0 - KH[1][1], -KH[1][2]],
            [-KH[2][0], -KH[2][1], 1.0 - KH[2][2]],
        ]
        self.cov = _mat_mul_3x3(I_KH, self.cov)

        self.total_corrections += 1
        if abs(wall.A) > 0.9:
            self.x_corrections += 1
            self.last_x_correction_time = time.time()

        self.lost_count = 0

    def should_reacquire(self):
        idle_time = time.time() - self.last_move_time
        if idle_time < 8.0:
            return False
        return self.lost_count > EKF_LOST_COUNT_MAX

    def get_covariance_diag(self):
        return self.cov[0][0], self.cov[1][1], self.cov[2][2]


def _mat_mul_3x3(A, B):
    return [
        [
            A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0],
            A[0][0] * B[0][1] + A[0][1] * B[1][1] + A[0][2] * B[2][1],
            A[0][0] * B[0][2] + A[0][1] * B[1][2] + A[0][2] * B[2][2],
        ],
        [
            A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0],
            A[1][0] * B[0][1] + A[1][1] * B[1][1] + A[1][2] * B[2][1],
            A[1][0] * B[0][2] + A[1][1] * B[1][2] + A[1][2] * B[2][2],
        ],
        [
            A[2][0] * B[0][0] + A[2][1] * B[1][0] + A[2][2] * B[2][0],
            A[2][0] * B[0][1] + A[2][1] * B[1][1] + A[2][2] * B[2][1],
            A[2][0] * B[0][2] + A[2][1] * B[1][2] + A[2][2] * B[2][2],
        ],
    ]


def _transpose_3x3(M):
    return [
        [M[0][0], M[1][0], M[2][0]],
        [M[0][1], M[1][1], M[2][1]],
        [M[0][2], M[1][2], M[2][2]],
    ]


def _mat_vec_mul(M, v):
    return [
        M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2],
        M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2],
        M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2],
    ]
