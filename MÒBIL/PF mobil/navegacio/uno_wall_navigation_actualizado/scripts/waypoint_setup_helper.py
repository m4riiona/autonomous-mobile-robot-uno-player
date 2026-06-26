#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import euler_from_quaternion


class WaypointSetupHelper:
    """Ayuda a obtener coordenadas para card_waypoints.yaml desde RViz."""

    def __init__(self):
        rospy.init_node("waypoint_setup_helper")

        self.last_goal = None
        rospy.Subscriber(
            "move_base_simple/goal", PoseStamped, self.goal_callback, queue_size=1
        )
        rospy.Service("/uno/print_last_goal", Trigger, self.handle_print_last_goal)

        rospy.loginfo(
            "Waypoint setup helper listo. Usa 2D Nav Goal en RViz y luego:"
        )
        rospy.loginfo("  rosservice call /uno/print_last_goal \"{}\"")

    def goal_callback(self, msg):
        self.last_goal = msg
        yaw = euler_from_quaternion(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
        )[2]
        rospy.loginfo(
            "Meta capturada: x=%.3f y=%.3f yaw=%.3f",
            msg.pose.position.x,
            msg.pose.position.y,
            yaw,
        )

    def handle_print_last_goal(self, _req):
        if self.last_goal is None:
            return TriggerResponse(
                success=False,
                message="No hay meta todavia. Pulsa 2D Nav Goal en RViz primero.",
            )

        yaw = euler_from_quaternion(
            [
                self.last_goal.pose.orientation.x,
                self.last_goal.pose.orientation.y,
                self.last_goal.pose.orientation.z,
                self.last_goal.pose.orientation.w,
            ]
        )[2]

        block = (
            "  - name: carta_N\n"
            "    x: %.3f\n"
            "    y: %.3f\n"
            "    yaw: %.3f"
        ) % (
            self.last_goal.pose.position.x,
            self.last_goal.pose.position.y,
            yaw,
        )

        rospy.loginfo("Copia esto en config/card_waypoints.yaml:\n%s", block)
        return TriggerResponse(success=True, message=block)


if __name__ == "__main__":
    try:
        WaypointSetupHelper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
