#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import math

class RobustFTGStable(Node):
    def __init__(self):
        super().__init__("robust_ftg_stable")

        # Subs & pubs
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/ego_racecar/odom", self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)

        # PID para steering (Ajustado para mayor agresividad y estabilidad a alta velocidad)
        self.Kp = 1.35  # Aumentado (antes 1.15) para respuesta más rápida al error
        self.Ki = 0.0
        self.Kd = 0.15  # Aumentado (antes 0.08) para amortiguar oscilaciones por Kp alto
        self.integral = 0.0
        self.prev_error = 0.0

        # Filtro sobre steering (low-pass)
        self.steer_alpha = 0.8  # Aumentado (antes 0.7) para suavizar la dirección a alta velocidad
        self.prev_steer = 0.0

        # Speed control (Muy agresivo en rectas, conservador en curvas)
        self.max_speed = 10.0 # Aumentado (antes 8.5) para maximizar la velocidad en rectas
        self.min_speed = 2.5
        self.k_speed = 7.5  # Aumentado (antes 6.0) para mayor velocidad base potencial

        # Límite de ángulo (rad)
        self.max_steer = 0.6    # ~34 deg

        # parámetros FTG / preprocesado (Se mantienen altos para la seguridad en curvas cerradas)
        self.max_range = 10.0
        self.smooth_window = 7
        self.free_thresh = 0.8
        self.inflation_bins = 18 
        
        # lookahead para apuntar a punto adelantado (evita correcciones bruscas)
        self.min_lookahead = 0.8
        self.max_lookahead = 2.5

        # Lap timing
        self.lap_count = 0
        self.lap_start_time = time.time()
        self.prev_in_start_zone = False
        self.has_left_start_zone = False
        self.lap_times = []

        self.get_logger().info("RobustFTGStable V2.0 initialized (Performance Optimized)")

    # ---------- UTIL ----------
    def smooth_ranges(self, ranges):
        r = np.array(ranges, dtype=np.float32)
        r[r == 0.0] = self.max_range
        r = np.minimum(r, self.max_range)
        # media móvil
        kernel = np.ones(self.smooth_window) / float(self.smooth_window)
        r_s = np.convolve(r, kernel, mode='same')
        return r_s

    def inflate_all_obstacles(self, ranges, free_thresh=None):
        """Crear versión inflada: cualquier punto con distancia < free_thresh se considera obstáculo y se expande."""
        if free_thresh is None:
            free_thresh = self.free_thresh
        r = ranges.copy()
        n = len(r)
        obstacle_mask = r < free_thresh
        inflated = np.ones(n, dtype=bool)  # True = free by default
        inflated[:] = True
        # mark obstacle points as False (not free)
        inflated[obstacle_mask] = False
        # expand False zones by inflation_bins
        for i in np.where(~inflated)[0]:
            lo = max(0, i - self.inflation_bins)
            hi = min(n, i + self.inflation_bins + 1)
            inflated[lo:hi] = False
        return inflated

    def find_gaps_from_mask(self, free_mask):
        """Return list of (start, end) indices of contiguous True segments in free_mask"""
        gaps = []
        start = None
        for i, v in enumerate(free_mask):
            if v and start is None:
                start = i
            elif (not v) and start is not None:
                gaps.append((start, i - 1))
                start = None
        if start is not None:
            gaps.append((start, len(free_mask) - 1))
        return gaps

    def compute_centroid_angle(self, ranges, gap, scan):
        """Compute a more stable target angle: centroid (weighted) of gap points in Cartesian coords"""
        g0, g1 = gap
        idxs = np.arange(g0, g1+1)
        dists = ranges[g0:g1+1]
        # remove any near-zero
        valid = dists > 0.01
        if not np.any(valid):
            return None
        idxs = idxs[valid]
        dists = dists[valid]
        # Convert to Cartesian (robot frame), x forward, y left
        angles = scan.angle_min + idxs * scan.angle_increment
        xs = dists * np.cos(angles)
        ys = dists * np.sin(angles)
        # weight: prefer farther points (square to accentuate)
        weights = dists**1.5
        xw = np.sum(xs * weights)
        yw = np.sum(ys * weights)
        wsum = np.sum(weights)
        if wsum == 0:
            return None
        x_cent = xw / wsum
        y_cent = yw / wsum
        # compute angle to centroid (atan2(y,x)), positive = left
        angle = math.atan2(y_cent, x_cent)
        # also return distance if needed
        dist = math.hypot(x_cent, y_cent)
        return angle, dist

    # ---------- CALLBACKS ----------
    def lidar_callback(self, scan: LaserScan):
        # preprocess
        ranges = self.smooth_ranges(scan.ranges)

        # compute inflated free mask
        free_mask = self.inflate_all_obstacles(ranges, free_thresh=self.free_thresh)

        # find gaps
        gaps = self.find_gaps_from_mask(free_mask)

        if not gaps:
            # si no hay gaps, frenar y mantener
            speed = 0.0
            steering = 0.0
            self.get_logger().warn_once("No gaps found - stopping briefly")
        else:
            # choose largest gap by width (prefer central gaps)
            gap_lengths = [g[1] - g[0] + 1 for g in gaps]
            best_idx = int(np.argmax(gap_lengths))
            best_gap = gaps[best_idx]

            # compute centroid angle and distance
            res = self.compute_centroid_angle(ranges, best_gap, scan)
            if res is None:
                # fallback: center of gap
                g0, g1 = best_gap
                mid = (g0 + g1) // 2
                target_angle = scan.angle_min + mid * scan.angle_increment
                target_dist = ranges[mid]
            else:
                target_angle, target_dist = res

            # PID on the target_angle (error = target_angle)
            error = float(target_angle)
            self.integral += error
            derivative = error - self.prev_error
            steering_command = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
            self.prev_error = error

            # clip steering command to reasonable range, then apply low-pass filter
            steering_command = float(np.clip(steering_command, -self.max_steer, self.max_steer))
            steering = self.steer_alpha * self.prev_steer + (1.0 - self.steer_alpha) * steering_command
            self.prev_steer = steering

            # speed law: reduce speed on tight steering or short frontal distance
            # frontal distance approx = target_dist * cos(target_angle) but ensure positive
            frontal_est = max(0.1, target_dist * math.cos(target_angle) if target_dist is not None else 1.0)
            
            # Nuevo factor de penalización de ángulo: 0..1.0
            angle_penalty = abs(steering) / self.max_steer # 0..1.0
            
            # Frontal bonus: Se ajusta la interpolación para alcanzar alta velocidad más rápido
            front_bonus = np.interp(frontal_est, [0.2, 0.5, 1.5, 3.0], [0.0, 0.5, 0.9, 1.0])
            
            base = self.k_speed * front_bonus
            
            # Aseguramos que la velocidad base esté por encima del mínimo.
            if base < self.min_speed:
                base = self.min_speed
                
            # Aplicar penalización: Reducción del 80% en giro máximo (seguridad garantizada)
            speed = base * (1.0 - 0.8 * angle_penalty) 
            
            # clip final de velocidad
            speed = float(np.clip(speed, self.min_speed, self.max_speed))

        # publish ackermann
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)
        self.drive_pub.publish(msg)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        in_zone = (abs(x) < 2.0 and abs(y) < 2.0)
        if in_zone and not self.prev_in_start_zone and self.has_left_start_zone:
            t = time.time() - self.lap_start_time
            self.lap_count += 1
            self.lap_times.append(t)
            self.get_logger().info(f"🏁 Vuelta {self.lap_count} — {t:.2f}s (Mejor: {min(self.lap_times):.2f}s)")
            self.lap_start_time = time.time()

        if not in_zone:
            self.has_left_start_zone = True
        self.prev_in_start_zone = in_zone


def main(args=None):
    rclpy.init(args=args)
    node = RobustFTGStable()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
