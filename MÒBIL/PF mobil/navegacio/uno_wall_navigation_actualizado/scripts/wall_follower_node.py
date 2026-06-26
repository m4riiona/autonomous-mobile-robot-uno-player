#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class RightWallFollower:
    def __init__(self):
        rospy.init_node("right_wall_follower")

        self.scan_topic = "/scan"
        self.cmd_vel_topic = "/cmd_vel"

        # Mas lejos de la pared para que no se pegue.
        self.target_distance = 0.40
        self.too_close_side = 0.25

        # Gira antes cuando ve pared delante.
        self.front_turn_distance = 0.60
        self.front_stop_distance = 0.32

        self.forward_speed = 0.055
        self.slow_speed = 0.035
        self.search_speed = 0.05
        self.turn_speed = 0.25
        self.max_angular = 0.32

        self.scan = None
        self.last_state = ""
        self.enabled = rospy.get_param("~enabled", True)

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber("~enable", Bool, self.enable_callback, queue_size=1)

        rospy.on_shutdown(self.stop)

        rospy.loginfo("RIGHT WALL FOLLOWER iniciado")
        rospy.loginfo("Sigue pared por la DERECHA a %.2f m", self.target_distance)

    def scan_callback(self, msg):
        self.scan = msg

    def enable_callback(self, msg):
        self.enabled = msg.data
        if not self.enabled:
            self.stop()
            self.set_state("Wall follower PAUSADO")

    def stop(self):
        self.cmd_pub.publish(Twist())

    def publish(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(max(-self.max_angular, min(self.max_angular, angular)))
        self.cmd_pub.publish(cmd)

    def valid(self, value):
        return (
            value is not None
            and not math.isnan(value)
            and not math.isinf(value)
            and self.scan.range_min < value < self.scan.range_max
        )

    def normalize_deg(self, angle):
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def sector_median(self, min_deg, max_deg):
        values = []

        for i, d in enumerate(self.scan.ranges):
            if not self.valid(d):
                continue

            angle = self.scan.angle_min + i * self.scan.angle_increment
            deg = self.normalize_deg(math.degrees(angle))

            if min_deg <= deg <= max_deg:
                values.append(d)

        if not values:
            return None

        values.sort()
        return values[len(values) // 2]

    def sector_min(self, min_deg, max_deg):
        values = []

        for i, d in enumerate(self.scan.ranges):
            if not self.valid(d):
                continue

            angle = self.scan.angle_min + i * self.scan.angle_increment
            deg = self.normalize_deg(math.degrees(angle))

            if min_deg <= deg <= max_deg:
                values.append(d)

        if not values:
            return None

        return min(values)

    def set_state(self, text):
        if text != self.last_state:
            rospy.loginfo(text)
            self.last_state = text

    def fmt(self, value):
        if value is None:
            return "None"
        return "%.2f" % value

    def spin(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if not self.enabled:
                self.stop()
                rate.sleep()
                continue

            if self.scan is None:
                self.set_state("Esperando /scan...")
                self.stop()
                rate.sleep()
                continue

            self.control()
            rate.sleep()

    def control(self):
        front = self.sector_min(-20, 20)
        front_right = self.sector_median(-65, -25)
        right = self.sector_median(-110, -70)
        back_right = self.sector_median(-145, -115)

        rospy.loginfo_throttle(
            0.5,
            "Laser | front=%s m | front_right=%s m | right=%s m | back_right=%s m",
            self.fmt(front),
            self.fmt(front_right),
            self.fmt(right),
            self.fmt(back_right),
        )

        if front is not None and front < self.front_stop_distance:
            self.set_state("Pared MUY cerca delante: giro izquierda en sitio")
            self.publish(0.0, self.turn_speed)
            return

        if front is not None and front < self.front_turn_distance:
            self.set_state("Pared delante: tomando esquina hacia la izquierda")
            self.publish(self.slow_speed, self.turn_speed)
            return

        # Nuevo comportamiento:
        # si no ve pared derecha, avanza recto hasta encontrar una.
        if right is None or right > 1.20:
            self.set_state("No veo pared derecha: avanzando recto hasta encontrar pared")
            self.publish(self.search_speed, 0.0)
            return

        if right < self.too_close_side:
            self.set_state("Demasiado cerca de pared derecha: separandome")
            self.publish(self.slow_speed, 0.25)
            return

        error = right - self.target_distance

        # Pared derecha:
        # right > target -> pared lejos -> girar derecha.
        # right < target -> pared cerca -> girar izquierda.
        angular = -1.2 * error

        if front_right is not None and right is not None and front_right < right - 0.04:
            angular += 0.12

        angular = max(-0.22, min(0.22, angular))

        self.set_state("Avanzando siguiendo pared por la derecha")
        self.publish(self.forward_speed, angular)


if __name__ == "__main__":
    try:
        RightWallFollower().spin()
    except rospy.ROSInterruptException:
        pass


