#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

# ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class ReactiveFollowGap:
    def __init__(self):
        # Topics & Subscriptions, Publishers
        drive_topic = '/nav'
        lidarscan_topic = '/scan'
        #drive_topic = '/vesc/ackermann_cmd_mux/input/navigation'
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
        
        self.prev_steering_angle = 0.0

    def preprocess_lidar(self, ranges, angle_increment):
        """ Preprocess the LiDAR scan array.
            1. Rejecting high values (eg. > 3m) [cite: 22]
            2. Setting bad values (NaN) to a safe distance
            3. DISPARITY EXTENDER: Étend les obstacles pour éviter de couper les virages
        """
        proc_ranges = np.array(ranges)

        # 1. Nettoyage 
        proc_ranges[np.isnan(proc_ranges)] = 10.0
        proc_ranges[np.isinf(proc_ranges)] = 10.0
        
        # On cap les distances
        proc_ranges[proc_ranges > 3.0] = 3.0

        # Paramètres pour élargir les coins
        car_width = 0.50
        threshold = 0.30  

        diffs = np.diff(proc_ranges)
        

        disparities = np.where(np.abs(diffs) > threshold)[0]

        for i in disparities:
            # i est l'index du saut
            val_current = proc_ranges[i]
            val_next = proc_ranges[i+1]
            
            # On détermine quel côté est le mur (le point le plus proche)
            if val_current < val_next:
                closer_val = val_current
                # Le mur est à gauche du saut, on étend DROITE
                extend_direction = 1 
                start_idx = i + 1
            else:
                closer_val = val_next
                # Le mur est à droite du saut, on étend GAUCHE
                extend_direction = -1
                start_idx = i


            angle_width = math.atan2(car_width, closer_val)
            num_indices_to_cover = int(angle_width / angle_increment)

            for k in range(num_indices_to_cover):
                idx_to_change = start_idx + (k * extend_direction)
                
                if 0 <= idx_to_change < len(proc_ranges):
                    proc_ranges[idx_to_change] = min(proc_ranges[idx_to_change], closer_val)

        return proc_ranges

    def find_max_gap(self, free_space_ranges):

        mask = free_space_ranges > 0

        d = np.diff(np.concatenate(([0], mask.view(np.int8), [0])))
        starts = np.where(d == 1)[0]
        ends = np.where(d == -1)[0]

        # Sécurité
        if len(starts) == 0:
            return 0, len(free_space_ranges) - 1

        # Trouver la séquence la plus longue
        lengths = ends - starts
        longest_idx = np.argmax(lengths)

        return starts[longest_idx], ends[longest_idx]

    def find_best_point(self, start_i, end_i, ranges):
        gap = ranges[start_i:end_i]
        
        # 2. On trouve la profondeur maximale dans ce gap
        max_dist = np.max(gap)
        
        # Cela crée un plateau de valeurs sûres
        safe_threshold = max_dist * 0.90
        
        best_indices = np.where(gap >= safe_threshold)[0]
        
        # 4. On vise le MILIEU
        if len(best_indices) > 0:
            avg_idx_relative = int(np.mean(best_indices))
        else:
            avg_idx_relative = int(len(gap) / 2)
        return start_i + avg_idx_relative

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm """
        
        raw_ranges = data.ranges
        
        proc_ranges = self.preprocess_lidar(raw_ranges, data.angle_increment)

        min_angle = -90 * math.pi / 180
        max_angle =  90 * math.pi / 180
        
        angle_inc = data.angle_increment
        idx_min_fov = int((min_angle - data.angle_min) / angle_inc)
        idx_max_fov = int((max_angle - data.angle_min) / angle_inc)
        
        # Sécurité bornes
        idx_min_fov = max(0, idx_min_fov)
        idx_max_fov = min(len(proc_ranges), idx_max_fov)
        
        # On découpe le tableau traité
        fov_ranges = proc_ranges[idx_min_fov : idx_max_fov]
        
        # --- Etape 1 : Trouver le point le plus proche [cite: 23] ---
        min_idx = np.argmin(fov_ranges)
        

        # On met 0 autour du point le plus proche
        bubble_radius = 0.60 
        radius_indices = int(bubble_radius / angle_inc)
        
        min_bubble = max(0, min_idx - radius_indices)
        max_bubble = min(len(fov_ranges), min_idx + radius_indices)
        
        fov_ranges[min_bubble : max_bubble] = 0

        start_i, end_i = self.find_max_gap(fov_ranges)

        # Centre du Plateau
        best_idx_relative = self.find_best_point(start_i, end_i, fov_ranges)

        real_best_idx = idx_min_fov + best_idx_relative
        target_steering_angle = data.angle_min + (real_best_idx * angle_inc)

        # LISSAGE (Anti-Zigzag) : Moyenne pondérée exponentielle

        alpha = 0.3
        smoothed_angle = (alpha * target_steering_angle) + ((1.0 - alpha) * self.prev_steering_angle)
        
        self.prev_steering_angle = smoothed_angle

        # Publication
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = smoothed_angle
        
        # Gestion de vitesse 
        if abs(smoothed_angle) > 20.0 * math.pi / 180.0:
            drive_msg.drive.speed = 0.5 
        else:
            drive_msg.drive.speed = 1.3 

        self.drive_pub.publish(drive_msg)

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = ReactiveFollowGap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)