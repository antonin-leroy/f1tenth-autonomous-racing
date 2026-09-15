#!/usr/bin/env python3
import rospy
import csv
import math
import numpy as np
import tf.transformations
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker

class PurePursuit(object):
    def __init__(self):
        rospy.loginfo("🚀 Pure Pursuit : Mode Transformation Locale")
        
        # --- PARAMETRES ---
        self.wheelbase = 0.33
        self.L = 1.2 
        self.max_steer = 0.4
        
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
        self.marker_pub = rospy.Publisher("/viz_target", Marker, queue_size=10)
        self.track_pub = rospy.Publisher("/viz_track", Marker, queue_size=10)

        # --- CHARGEMENT CSV ---
        self.waypoints = []
        csv_file = '/home/antonin/catkin_ws/src/f1tenth_labs/waypoint_logger/scripts/waypoints.csv'

        try:
            with open(csv_file, 'r') as csvfile:
                csv_reader = csv.reader(csvfile)
                for row in csv_reader:
                    try:
                        self.waypoints.append([float(row[0]), float(row[1])])
                    except ValueError:
                        continue
            self.waypoints = np.array(self.waypoints)
            print(f"✅ Chargé {len(self.waypoints)} points.")
            self.publish_track()
        except FileNotFoundError:
            print(f"❌ Fichier introuvable : {csv_file}")
            exit()

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.pose_callback)
        rospy.Timer(rospy.Duration(2.0), self.publish_track)

    def publish_track(self, event=None):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = 999; marker.type = marker.LINE_STRIP; marker.action = marker.ADD
        marker.scale.x = 0.05; marker.color.a = 1.0; marker.color.b = 1.0
        
        step = 5 
        for i in range(0, len(self.waypoints), step):
            p = self.waypoints[i]
            pt = Point(); pt.x = p[0]; pt.y = p[1]
            marker.points.append(pt)
        self.track_pub.publish(marker)

    def pose_callback(self, data):
        x_car = data.pose.pose.position.x
        y_car = data.pose.pose.position.y
        q = data.pose.pose.orientation
        (_, _, yaw) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        car_pos = np.array([x_car, y_car])

        dists = np.linalg.norm(self.waypoints - car_pos, axis=1)
        closest_index = np.argmin(dists)

        # TRANSFORMATION LOCALE
        indices = [(closest_index + i) % len(self.waypoints) for i in range(100)]
        local_chunk = self.waypoints[indices]

        # X_local = Devant, Y_local = Gauche
        dx = local_chunk[:, 0] - x_car
        dy = local_chunk[:, 1] - y_car

        x_local = dx * math.cos(-yaw) - dy * math.sin(-yaw)
        y_local = dx * math.sin(-yaw) + dy * math.cos(-yaw)

        # On cherche le premier point qui remplit DEUX :
        # -Il est DEVANT la voiture (x_local > 0)
        # -Il est à la distance L (ou vient de la dépasser)
        
        target_found = False
        target_x = 0.0
        target_y = 0.0

        for i in range(len(indices)):
            lx = x_local[i]
            ly = y_local[i]
            dist = math.sqrt(lx**2 + ly**2)

            # Si le point est derrière, on l'ignore (c'est ça qui empêche les demi-tours !)
            if lx < 0:
                continue

            # Si le point est devant et qu'il dépasse la distance L
            if dist >= self.L:
                target_x = lx
                target_y = ly
                target_found = True
                
                # Visualisation du point choisi (dans le repère Map pour Rviz)
                real_target = self.waypoints[indices[i]]
                self.viz_target(real_target)
                break
        
        # SÉCURITÉ : Si on coupe le virage et que TOUS les points sont < L mais devant
        if not target_found:
             # On cherche n'importe quel point devant, le plus loin possible
             valid_indices = np.where(x_local > 0)[0]
             if len(valid_indices) > 0:
                 last_valid = valid_indices[-1]
                 target_x = x_local[last_valid]
                 target_y = y_local[last_valid]
                 self.viz_target(self.waypoints[indices[last_valid]])
             else:
                 pass

        
        gamma = (2 * target_y) / (self.L**2)
        steer = math.atan(gamma * self.wheelbase)
        steer = np.clip(steer, -self.max_steer, self.max_steer)

        # Vitesse
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.drive.steering_angle = steer
        msg.drive.speed = 1.5 if abs(steer) > 0.2 else 2.5
            
        self.drive_pub.publish(msg)

    def viz_target(self, p):
        m = Marker()
        m.header.frame_id = 'map'
        m.id = 0; m.type = m.SPHERE; m.action = m.ADD
        m.pose.position.x = p[0]
        m.pose.position.y = p[1]
        m.scale.x = 0.4; m.scale.y = 0.4; m.scale.z = 0.4
        m.color.a = 1.0; m.color.g = 1.0 
        m.pose.orientation.w = 1.0
        self.marker_pub.publish(m)

def main():
    rospy.init_node('pure_pursuit_node')
    PurePursuit()
    rospy.spin()

if __name__ == '__main__':
    main()