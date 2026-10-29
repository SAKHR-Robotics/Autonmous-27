"""Measurement-timed planar wheel odometry; no ROS dependencies."""
import math


class WheelOdometry:
    def __init__(self, names, meters_per_tick, track_width, timeout=0.5,
                 max_speed=2.0, slip_threshold=0.15):
        if (len(set(names)) != len(names) or not names
                or not any('left' in n for n in names)
                or not any('right' in n for n in names)):
            raise ValueError('Unique left/right wheel names required')
        if not all(math.isfinite(v) and v > 0 for v in
                   (meters_per_tick, track_width, timeout, max_speed, slip_threshold)):
            raise ValueError('Positive finite wheel geometry, limits and timeout required')
        self.names = list(names)
        self.scale, self.track_width = meters_per_tick, track_width
        self.timeout, self.max_speed, self.slip_threshold = timeout, max_speed, slip_threshold
        self.counts, self.stamp = None, None
        self.x = self.y = self.yaw = 0.0
        self.reason = 'waiting for complete wheel input'

    def reset_measurements(self):
        # Preserve pose; discard differences across a clock jump or sensor restart.
        self.counts, self.stamp = None, None
        self.reason = 'measurement baseline reset'

    def observe(self, counts, stamp):
        from rover_slam.encoder_ticks_to_odom import compute_robust_side_velocity
        if set(counts) != set(self.names) or not math.isfinite(stamp):
            self.reason = 'incomplete or invalid wheel sample'
            return None
        if not all(isinstance(c, (int,float)) and math.isfinite(c) for c in counts.values()):
            self.reason = 'non-finite wheel sample'
            return None
        if self.stamp is None:
            self.counts, self.stamp = dict(counts), stamp
            self.reason = 'waiting for second wheel sample'
            return None
        dt = stamp - self.stamp
        if dt <= 0:
            self.reason = 'duplicate or out-of-order wheel sample'
            return None
        speeds = {n: (counts[n]-self.counts[n])*self.scale/dt for n in self.names}
        self.counts, self.stamp = dict(counts), stamp
        if dt > self.timeout or any(abs(v) > self.max_speed for v in speeds.values()):
            self.reason = 'wheel gap or counter discontinuity; baseline reset'
            return None
        left, slip_l = compute_robust_side_velocity(
            [v for n,v in speeds.items() if 'left' in n], self.slip_threshold)
        right, slip_r = compute_robust_side_velocity(
            [v for n,v in speeds.items() if 'right' in n], self.slip_threshold)
        velocity, yaw_rate = (left+right)/2, (right-left)/self.track_width
        distance, angle = velocity*dt, yaw_rate*dt
        # Exact constant-curvature integration, including the straight-line limit.
        if abs(angle) < 1e-8:
            self.x += distance*math.cos(self.yaw+angle/2)
            self.y += distance*math.sin(self.yaw+angle/2)
        else:
            self.x += distance/angle*(math.sin(self.yaw+angle)-math.sin(self.yaw))
            self.y += distance/angle*(math.cos(self.yaw)-math.cos(self.yaw+angle))
        self.yaw = math.atan2(math.sin(self.yaw+angle), math.cos(self.yaw+angle))
        self.reason = 'healthy'
        return velocity, yaw_rate, speeds, slip_l or slip_r
