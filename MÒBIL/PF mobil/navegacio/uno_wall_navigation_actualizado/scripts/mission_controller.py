#!/usr/bin/env python3

import threading

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import quaternion_from_euler


class Waypoint(object):
    def __init__(self, name, x, y, yaw):
        self.name = name
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)


class MissionController:
    def __init__(self):
        rospy.init_node("uno_mission_controller")

        self.map_frame = rospy.get_param("~map_frame", "map")
        self.move_base_timeout = rospy.Duration(
            rospy.get_param("~move_base_timeout", 120.0)
        )
        self.stop_delay = rospy.get_param("~stop_delay", 0.5)

        self.waypoints = self.load_waypoints()
        self.current_index = -1
        self.patrol_active = False
        self.waiting_at_waypoint = False
        self.mission_lock = threading.Lock()
        self.mission_running = False

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.wall_enable_pub = rospy.Publisher(
            "/uno_wall_follower/enable", Bool, queue_size=1, latch=True
        )
        self.status_pub = rospy.Publisher(
            "/uno/mission_status", String, queue_size=1, latch=True
        )
        self.current_waypoint_pub = rospy.Publisher(
            "/uno/current_waypoint_index", Int32, queue_size=1, latch=True
        )
        self.current_pose_pub = rospy.Publisher(
            "/uno/current_waypoint_pose", PoseStamped, queue_size=1, latch=True
        )

        rospy.Service("/uno/start_patrol", Trigger, self.handle_start_patrol)
        rospy.Service("/uno/continue_patrol", Trigger, self.handle_continue_patrol)
        rospy.Service("/uno/stop_patrol", Trigger, self.handle_stop_patrol)
        rospy.Service("/uno/resume_wall_follow", Trigger, self.handle_resume_wall)

        rospy.loginfo("Esperando move_base...")
        self.move_base.wait_for_server()
        rospy.loginfo("Mission controller listo con %d waypoints", len(self.waypoints))
        for i, wp in enumerate(self.waypoints):
            rospy.loginfo(
                "  [%d] %s -> x=%.2f y=%.2f yaw=%.2f",
                i,
                wp.name,
                wp.x,
                wp.y,
                wp.yaw,
            )
        rospy.loginfo(
            "Servicios: /uno/start_patrol, /uno/continue_patrol, "
            "/uno/stop_patrol, /uno/resume_wall_follow"
        )

        self.publish_status("idle")

    def load_waypoints(self):
        raw = rospy.get_param("~waypoints", [])
        if not raw:
            rospy.logwarn("No hay waypoints en ~waypoints; lista vacia")
            return []

        waypoints = []
        for entry in raw:
            waypoints.append(
                Waypoint(
                    entry.get("name", "carta_%d" % len(waypoints)),
                    entry["x"],
                    entry["y"],
                    entry.get("yaw", 0.0),
                )
            )
        return waypoints

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))

    def publish_current_waypoint(self, index):
        self.current_waypoint_pub.publish(Int32(data=index))
        if 0 <= index < len(self.waypoints):
            wp = self.waypoints[index]
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = wp.x
            pose.pose.position.y = wp.y
            q = quaternion_from_euler(0.0, 0.0, wp.yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            self.current_pose_pub.publish(pose)

    def set_wall_follower(self, enabled):
        self.wall_enable_pub.publish(Bool(data=enabled))
        if not enabled:
            self.cmd_pub.publish(Twist())

    def make_goal(self, waypoint):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = waypoint.x
        goal.target_pose.pose.position.y = waypoint.y
        goal.target_pose.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, waypoint.yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        return goal

    def send_goal_and_wait(self, goal):
        self.move_base.send_goal(goal)
        finished = self.move_base.wait_for_result(self.move_base_timeout)
        if not finished:
            self.move_base.cancel_goal()
            return False, "timeout"

        state = self.move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            return True, "ok"

        return False, "estado %s" % state

    def go_to_waypoint(self, index):
        if index < 0 or index >= len(self.waypoints):
            return False, "Indice de waypoint invalido: %d" % index

        wp = self.waypoints[index]
        rospy.loginfo(
            "Yendo a [%d] %s (%.2f, %.2f, %.2f)",
            index,
            wp.name,
            wp.x,
            wp.y,
            wp.yaw,
        )
        self.publish_status("navegando_a_%s" % wp.name)
        ok, detail = self.send_goal_and_wait(self.make_goal(wp))
        if not ok:
            self.publish_status("error_navegacion")
            return False, "Fallo al ir a %s: %s" % (wp.name, detail)

        rospy.sleep(self.stop_delay)
        self.cmd_pub.publish(Twist())
        self.current_index = index
        self.waiting_at_waypoint = True
        self.publish_current_waypoint(index)
        self.publish_status("esperando_en_%s" % wp.name)
        rospy.loginfo(
            "Parado en [%d] %s. Llama a /uno/continue_patrol para seguir.",
            index,
            wp.name,
        )
        return True, "Parado en [%d] %s" % (index, wp.name)

    def run_start_patrol(self):
        if not self.waypoints:
            return False, "No hay waypoints definidos"

        self.set_wall_follower(False)
        rospy.sleep(self.stop_delay)
        self.patrol_active = True
        return self.go_to_waypoint(0)

    def run_continue_patrol(self):
        if not self.patrol_active:
            return False, "No hay patrulla activa. Usa /uno/start_patrol primero."

        if not self.waiting_at_waypoint:
            return False, "El robot aun no ha llegado a un waypoint"

        next_index = self.current_index + 1
        if next_index >= len(self.waypoints):
            self.waiting_at_waypoint = False
            self.patrol_active = False
            self.publish_status("patrulla_completa")
            return True, "Patrulla completa: visitados %d waypoints" % len(
                self.waypoints
            )

        self.waiting_at_waypoint = False
        return self.go_to_waypoint(next_index)

    def run_in_thread(self, func):
        with self.mission_lock:
            if self.mission_running:
                return False, "Ya hay una mision en curso"
            self.mission_running = True

        result = {"ok": False, "message": ""}

        def worker():
            try:
                ok, message = func()
                result["ok"] = ok
                result["message"] = message
            finally:
                with self.mission_lock:
                    self.mission_running = False

        thread = threading.Thread(target=worker)
        thread.daemon = True
        thread.start()
        thread.join()
        return result["ok"], result["message"]

    def handle_start_patrol(self, _req):
        ok, message = self.run_in_thread(self.run_start_patrol)
        return TriggerResponse(success=ok, message=message)

    def handle_continue_patrol(self, _req):
        ok, message = self.run_in_thread(self.run_continue_patrol)
        return TriggerResponse(success=ok, message=message)

    def handle_stop_patrol(self, _req):
        self.move_base.cancel_goal()
        self.cmd_pub.publish(Twist())
        self.patrol_active = False
        self.waiting_at_waypoint = False
        self.current_index = -1
        self.publish_status("patrulla_detenida")
        return TriggerResponse(success=True, message="Patrulla detenida")

    def handle_resume_wall(self, _req):
        self.patrol_active = False
        self.waiting_at_waypoint = False
        self.set_wall_follower(True)
        self.publish_status("wall_follow")
        return TriggerResponse(success=True, message="Wall follower reanudado")


if __name__ == "__main__":
    try:
        MissionController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
