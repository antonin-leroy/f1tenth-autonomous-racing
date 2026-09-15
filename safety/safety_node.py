#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool

class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        """
        Initialisation des Publishers et Subscribers
        """
        self.speed = 0.0
        
        # TODO: create ROS subscribers and publishers.
        self.pub_brake = rospy.Publisher('/brake', AckermannDriveStamped, queue_size=10)
        #self.pub_brake = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/safety', AckermannDriveStamped, queue_size=10)
        self.pub_bool = rospy.Publisher('/brake_bool', Bool, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        distances = scan_msg.ranges
        angle_min = scan_msg.angle_min
        incr = scan_msg.angle_increment
        
        # Seuil de déclenchement
        TTC_THRESHOLD = 0.5

        for i in range(len(distances)):
            dist = distances[i]
            
            # Sécurité 
            if math.isinf(dist) or math.isnan(dist):
                continue
            
            # Calcul de l'angle et de la vitesse proj
            alpha = angle_min + (i * incr)
            vitesse_proj = self.speed * math.cos(alpha)
            
            if vitesse_proj <= 0:
                continue
            
            ttc = dist / vitesse_proj
            
            # TODO: publish brake message and publish controller bool
            if ttc < TTC_THRESHOLD:
                brake_msg = AckermannDriveStamped()
                brake_msg.drive.speed = 0.0
                bool_msg = Bool()
                bool_msg.data = True
                
                # --- Envoi ---
                self.pub_brake.publish(brake_msg)
                self.pub_bool.publish(bool_msg)
                
                # On a trouvé un danger, on arrête la boucle pour ne pas spammer
                break

def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()

if __name__ == '__main__':
    main()