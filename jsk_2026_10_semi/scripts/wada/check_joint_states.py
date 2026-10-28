#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

target_joints = [
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

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('joint_states')
        self.subscription=self.create_subscription(JointState,'joint_states',self.listener_callback,10)
    def listener_callback(self,msg):
        #self.get_logger().info(f'I heard: "{msg.data}"')
        final_output="alltarget.append({"
        joint_angles = dict(zip(msg.name,msg.position))
        #使用しやすいように整形する
        for joint in target_joints:
            if joint in joint_angles:
                #self.get_logger().info("%s: %.2f" % (joint,joint_angles[joint]))
                final_output+='"'+joint+'"'+" :"+str("%.2f"% (joint_angles[joint]))+","
        final_output=final_output[:-1]
        final_output+="})"
        print(final_output)
        return

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber=MinimalSubscriber()
    rclpy.spin_once(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()