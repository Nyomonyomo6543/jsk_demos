import math
import time
import rclpy

from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node

from typing import Dict, Iterable, List

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

#Right Hand Joints
RIGHT_JOINTS = [
    "R_thumb_proximal_yaw_joint",
    "R_thumb_proximal_pitch_joint",
    "R_index_proximal_joint",
    "R_middle_proximal_joint",
    "R_ring_proximal_joint",
    "R_pinky_proximal_joint",
]
#Left Hand Joints
LEFT_JOINTS = [
    "L_thumb_proximal_yaw_joint",
    "L_thumb_proximal_pitch_joint",
    "L_index_proximal_joint",
    "L_middle_proximal_joint",
    "L_ring_proximal_joint",
    "L_pinky_proximal_joint",
]
#Hand Joints value
OPEN_POSITIONS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
CLOSED_POSITIONS = [0.3, 0.6, 1.7, 1.7, 1.7, 1.7]
#Upper body Joints
JOINT_NAMES = [
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]

alltarget=[]

#手を広げる
alltarget.append({"waist_yaw_joint" :0.00,"waist_roll_joint" :0.00,"waist_pitch_joint" :0.00,"left_shoulder_pitch_joint" :-0.35,"left_shoulder_roll_joint" :0.58,"left_shoulder_yaw_joint" :0.46,"left_elbow_joint" :1.03,"left_wrist_roll_joint" :-0.37,"left_wrist_pitch_joint" :-0.41,"left_wrist_yaw_joint" :-0.72,"right_shoulder_pitch_joint" :-0.32,"right_shoulder_roll_joint" :-0.56,"right_shoulder_yaw_joint" :-0.46,"right_elbow_joint" :0.95,"right_wrist_roll_joint" :0.37,"right_wrist_pitch_joint" :-0.37,"right_wrist_yaw_joint" :0.69})
#手を広げるpart2
#alltarget.append({"waist_yaw_joint" :-0.00,"waist_roll_joint" :-0.00,"waist_pitch_joint" :0.00,"left_shoulder_pitch_joint" :-0.31,"left_shoulder_roll_joint" :0.93,"left_shoulder_yaw_joint" :0.29,"left_elbow_joint" :0.69,"left_wrist_roll_joint" :-1.75,"left_wrist_pitch_joint" :0.68,"left_wrist_yaw_joint" :0.03,"right_shoulder_pitch_joint" :-0.32,"right_shoulder_roll_joint" :-0.56,"right_shoulder_yaw_joint" :-0.46,"right_elbow_joint" :0.95,"right_wrist_roll_joint" :0.37,"right_wrist_pitch_joint" :-0.37,"right_wrist_yaw_joint" :0.69})
#手を差し込む体制へ
#より平行に近い形にした

alltarget.append({"waist_yaw_joint" :0.00,"waist_roll_joint" :-0.00,"waist_pitch_joint" :0.00,"left_shoulder_pitch_joint" :0.92,"left_shoulder_roll_joint" :-0.06,"left_shoulder_yaw_joint" :0.89,"left_elbow_joint" :0.10,"left_wrist_roll_joint" :-1.07,"left_wrist_pitch_joint" :1.08,"left_wrist_yaw_joint" :-0.97,"right_shoulder_pitch_joint" :0.20,"right_shoulder_roll_joint" :-0.36,"right_shoulder_yaw_joint" :-0.42,"right_elbow_joint" :0.12,"right_wrist_roll_joint" :0.42,"right_wrist_pitch_joint" :-0.14,"right_wrist_yaw_joint" :0.49})

#手を差し込む

alltarget.append({"waist_yaw_joint" :0.00,"waist_roll_joint" :-0.00,"waist_pitch_joint" :0.00,"left_shoulder_pitch_joint" :-0.48,"left_shoulder_roll_joint" :0.15,"left_shoulder_yaw_joint" :0.78,"left_elbow_joint" :0.75,"left_wrist_roll_joint" :-1.22,"left_wrist_pitch_joint" :0.56,"left_wrist_yaw_joint" :0.07,"right_shoulder_pitch_joint" :0.91,"right_shoulder_roll_joint" :-0.14,"right_shoulder_yaw_joint" :-0.62,"right_elbow_joint" :0.03,"right_wrist_roll_joint" :0.77,"right_wrist_pitch_joint" :-0.55,"right_wrist_yaw_joint" :1.06})

#手を閉じて

#上にあげて袋を傾ける

alltarget.append({"waist_yaw_joint" :0.00,"waist_roll_joint" :0.00,"waist_pitch_joint" :0.00,"left_shoulder_pitch_joint" :-0.45,"left_shoulder_roll_joint" :0.57,"left_shoulder_yaw_joint" :-0.38,"left_elbow_joint" :0.71,"left_wrist_roll_joint" :-0.93,"left_wrist_pitch_joint" :1.20,"left_wrist_yaw_joint" :-0.80,"right_shoulder_pitch_joint" :0.20,"right_shoulder_roll_joint" :-0.36,"right_shoulder_yaw_joint" :-0.42,"right_elbow_joint" :0.12,"right_wrist_roll_joint" :0.42,"right_wrist_pitch_joint" :-0.14,"right_wrist_yaw_joint" :0.49})


#右手をいい感じに持っていく
alltarget.append({"waist_yaw_joint" :-0.00,"waist_roll_joint" :-0.00,"waist_pitch_joint" :-0.00,"left_shoulder_pitch_joint" :-0.95,"left_shoulder_roll_joint" :1.08,"left_shoulder_yaw_joint" :-0.79,"left_elbow_joint" :0.72,"left_wrist_roll_joint" :-1.50,"left_wrist_pitch_joint" :1.56,"left_wrist_yaw_joint" :-0.59,"right_shoulder_pitch_joint" :-0.30,"right_shoulder_roll_joint" :-0.39,"right_shoulder_yaw_joint" :0.00,"right_elbow_joint" :-0.54,"right_wrist_roll_joint" :0.55,"right_wrist_pitch_joint" :0.77,"right_wrist_yaw_joint" :0.56})
#初期状態    
alltarget.append({"waist_yaw_joint" :0.00,"waist_roll_joint" :0.00,"waist_pitch_joint" :0.00,"left_shoulder_pitch_joint" :0.00,"left_shoulder_roll_joint" :0.00,"left_shoulder_yaw_joint" :0.00,"left_elbow_joint" :0.00,"left_wrist_roll_joint" :0.00,"left_wrist_pitch_joint" :0.00,"left_wrist_yaw_joint" :0.00,"right_shoulder_pitch_joint" :0.00,"right_shoulder_roll_joint" :0.00,"right_shoulder_yaw_joint" :0.00,"right_elbow_joint" :0.00,"right_wrist_roll_joint" :0.00,"right_wrist_pitch_joint" :0.00,"right_wrist_yaw_joint" :0.00})



def validate_positive(name: str, value: float) -> float:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be a positive finite value, got {value}")
    return value

#ハンド開閉系のクラス
class InspireDFXHandOpenClose(Node):
    def __init__(self) -> None:
        super().__init__("hand_controllers")
        self.declare_parameter("hand", "both")
        self.declare_parameter("right_action_name", "/right_hand_controller/follow_joint_trajectory")
        self.declare_parameter("left_action_name", "/left_hand_controller/follow_joint_trajectory")
        self.declare_parameter("period_sec", 1.0)
        self.declare_parameter("move_duration_sec", 0.5)
        self.declare_parameter("server_timeout_sec", 10.0)
        self.declare_parameter("start_open", True)
        self.hand = self.get_parameter("hand").value.lower()
        self.move_duration_sec = validate_positive("move_duration_sec", float(self.get_parameter("move_duration_sec").value))
        #この中から一つ選ばせる
        if self.hand not in ("right", "left", "both"):
            raise ValueError("hand must be one of: right, left, both")

        self.action_clients: Dict[str, ActionClient] = {}
        self.action_names: Dict[str, str] = {}
        if self.hand in ("right", "both"):
            self.action_names["right"] = self.get_parameter("right_action_name").value
            self.action_clients["right"] = ActionClient(
                self,
                FollowJointTrajectory,
                self.action_names["right"],
            )
        if self.hand in ("left", "both"):
            self.action_names["left"] = self.get_parameter("left_action_name").value
            self.action_clients["left"] = ActionClient(
                self,
                FollowJointTrajectory,
                self.action_names["left"],
            )

        self.wait_for_action_servers()

        self.get_logger().info("Started RH56DFX open/close: hand=%s, move_duration=%.3fs"% (self.hand,self.move_duration_sec))

    #サーバの応答を待つ
    def wait_for_action_servers(self) -> None:
        for side, client in self.action_clients.items():
            if not client.wait_for_server(timeout_sec=5):
                raise RuntimeError(f"{side} hand action server is not available: {self.action_names[side]}")

    #コールバック関数
    def timer_callback(self) -> None:
        open_goal = self.open_next
        self.open_next = not self.open_next

        goal_handles = []
        for side, client in self.action_clients.items():
            joint_names = RIGHT_JOINTS if side == "right" else LEFT_JOINTS
            goal_msg = (
                self.make_open_goal(joint_names)
                if open_goal
                else self.make_close_goal(joint_names))
        future = client.send_goal_async(goal_msg)
        goal_handles.append((side, future))

        for side, future in goal_handles:
            future.add_done_callback(
                lambda done_future, side=side, label=label: self.goal_response_callback(
                    side, label, done_future
                )
            )

        self.get_logger().info("Sent %s command to %s hand(s)" % (label, self.hand))


    def hand_callback(self, hand: str, action: str) -> None:
        if hand not in ("right", "left", "both"):
            raise ValueError("hand must be right, left or both")
        if action not in ("open", "close"):
            raise ValueError("action must be open or close")

        sides = ["right", "left"] if hand == "both" else [hand]

        for side in sides:
            client = self.action_clients[side]
            joint_names = RIGHT_JOINTS if side == "right" else LEFT_JOINTS
            if action == "open":
                goal_msg = self.make_open_goal(joint_names)
            else:
                goal_msg = self.make_close_goal(joint_names)
            future = client.send_goal_async(goal_msg)
            future.add_done_callback(lambda done_future, side=side, action=action:self.goal_response_callback(side, action, done_future))
            self.get_logger().info(f"Sent {action} command to {side} hand")

    
    def make_point(self, positions: Iterable[float], time_from_start_sec: float) -> JointTrajectoryPoint:
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start.sec = int(time_from_start_sec)
        point.time_from_start.nanosec = int(
            round((time_from_start_sec - point.time_from_start.sec) * 1e9)
        )
        if point.time_from_start.nanosec >= 1000000000:
            point.time_from_start.sec += 1
            point.time_from_start.nanosec -= 1000000000
        return point

    #目標の位置を決定
    def make_goal(self, joint_names: Iterable[str], points: Iterable[JointTrajectoryPoint]) -> FollowJointTrajectory.Goal:
        trajectory = JointTrajectory()
        trajectory.joint_names = list(joint_names)
        trajectory.points = list(points)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        return goal

    #開くときのゴール
    def make_open_goal(self, joint_names: Iterable[str]) -> FollowJointTrajectory.Goal:
        ordered_joint_names = list(joint_names)
        point = self.make_point(OPEN_POSITIONS, self.move_duration_sec)
        return self.make_goal(ordered_joint_names, [point])

    #閉じるときのゴール
    def make_close_goal(self, joint_names: Iterable[str]) -> FollowJointTrajectory.Goal:
        ordered_joint_names = list(joint_names)
        point = self.make_point(CLOSED_POSITIONS, self.move_duration_sec)
        return self.make_goal(ordered_joint_names, [point])

    #ゴールにいけるか
    def goal_response_callback(self, side: str, label: str, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("%s %s goal was rejected" % (side, label))
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda done_future, side=side, label=label: self.result_callback(side, label, done_future))

    #結果のコールバック
    def result_callback(self, side: str, label: str, future) -> None:
        result = future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error("%s %s goal failed: error_code=%d, error_string=%s"% (side, label, result.error_code, result.error_string))


#上半身系
class Upper_Body_Move(Node):
    def __init__(self) -> None:
        super().__init__('upper_body_move')
        #self = Node("upper_body_move")
        #gear_sonic_controller
        #upper_body_controller
        #self.controller = self.declare_parameter("controller", "upper_body_controller").value
        self.controller = self.declare_parameter("controller", "gear_sonic_controller").value
        self.client = ActionClient(self, FollowJointTrajectory, f"/{self.controller}/follow_joint_trajectory")

    def make_point(self,current, positions_by_name, secc):
        point = JointTrajectoryPoint()
        point.positions = [positions_by_name.get(name, current[name]) for name in JOINT_NAMES]
        point.time_from_start = Duration(seconds=secc).to_msg()  # secc may be a float
        return point

    def upper_body_callback(self, target) -> None:
        self.get_logger().info(f"Waiting for {self.controller} ...")    
        self.client.wait_for_server()
        
        latest = {}
        self.create_subscription(JointState, "/joint_states", lambda msg: latest.update(zip(msg.name, msg.position)), 10)
        self.get_logger().info("Reading current joint positions ...")
        while rclpy.ok() and not all(name in latest for name in JOINT_NAMES):
            rclpy.spin_once(self, timeout_sec=0.1)
            current = dict(latest)  # starting posture
        for name, position in target.items():
            self.get_logger().info("%s: %.2f -> %.2f" % (name, current[name], position))

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        goal.trajectory.points = [
            self.make_point(current,{},0.1),
            self.make_point(current,target,1.5)
            ]
        
        self.get_logger().info("Sending goal")
        goal_handle = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_handle)
        if not goal_handle.result().accepted:
            self.get_logger().error("Goal rejected")
            self.destroy_node()
            rclpy.shutdown()
            return
        result = goal_handle.result().get_result_async()
        # rclpy.spin_until_future_complete(node, result)
        # print the current posture of the moving joints every 0.5 s until done
        next_print = self.get_clock().now()

        while rclpy.ok() and not result.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now() >= next_print:
                self.get_logger().info("now: " + ", ".join("%s %.2f" % (name, latest[name]) for name in target))
                next_print += Duration(seconds=0.5)
        self.get_logger().info("Done: error_code=%d %s"% (result.result().result.error_code, result.result().result.error_string))


def main(args=None):
    rclpy.init()
    hand_node = None
    upper_body_node =None
    upper_body_node=Upper_Body_Move()
    hand_node = InspireDFXHandOpenClose()

    hand_node.hand_callback("left", "open")
    rclpy.spin_once(hand_node)
    hand_node.hand_callback("right", "close")
    rclpy.spin_once(hand_node)

    #upper_body_node.upper_body_callback(alltarget[4])
    #rclpy.spin_once(upper_body_node)

    time.sleep(1)
    #main処理は以下
    for i in range(3):
        upper_body_node.upper_body_callback(alltarget[i])
        rclpy.spin_once(upper_body_node)

    hand_node.hand_callback("left", "close")
    rclpy.spin_once(hand_node)
    
    time.sleep(1)

    for i in range(2):
        upper_body_node.upper_body_callback(alltarget[i+3])
        rclpy.spin_once(upper_body_node)

    hand_node.hand_callback("left", "open")
    rclpy.spin_once(hand_node)
    
    time.sleep(1)
    
    upper_body_node.upper_body_callback(alltarget[5])
    rclpy.spin_once(upper_body_node)

    hand_node.hand_callback("left", "close")
    rclpy.spin_once(hand_node)
    time.sleep(1)
    upper_body_node.destroy_node()
    hand_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()