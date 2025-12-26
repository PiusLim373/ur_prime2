#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import TwistStamped, PoseStamped, Wrench, WrenchStamped, Pose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from control_msgs.msg import JointJog
from scipy.spatial.transform import Rotation as R
import time
from prime2_master.srv import *


def list_to_pose(list):
    data = Pose()
    data.position.x = list[0]
    data.position.y = list[1]
    data.position.z = list[2]
    data.orientation.x = list[3]
    data.orientation.y = list[4]
    data.orientation.z = list[5]
    data.orientation.w = list[6]
    return data


CHECK_CHARUCO_POSE = list_to_pose([0.45, 0.7, 2.00, 0.707, -0.707, 0.0, 0.0])  # xyzxyzw
CHARUCO_SERVO_OFFSET = [0.0, 0.0, 0.3, 0.0, 0.0, 0.0]  # xyzrpy
RACK_HOME = list_to_pose([0.0, 0.2, -0.33, 0.0, 0.0, 0.0, 1.0])
START_FT_SERVO_POSE = list_to_pose([0.0, -0.08, -0.33, 0.0, 0.0, 0.0, 1.0])
THEORITICAL_FOUP_PICK_POSE = list_to_pose([0.0, -0.27, -0.33, 0.0, 0.0, 0.0, 1.0])
FOUP_LIFT_UP = list_to_pose([0.0, -0.27, -0.40, 0.0, 0.0, 0.0, 1.0])
FOUP_LIFT_UP_RETRACT = list_to_pose([0.0, 0.2, -0.40, 0.0, 0.0, 0.0, 1.0])
SHELF_OUT_POSE = list_to_pose([0.0, 0.55, 1.36, 0.0, 1.0, 0.0, 0.0])
SHELF_IN_POSE = list_to_pose([0.0, 0.18, 1.36, 0.0, 1.0, 0.0, 0.0])
SHELF_PP_CONTACT = list_to_pose([0.0, 0.18, 1.35, 0.0, 1.0, 0.0, 0.0])
SHELF_PP = list_to_pose([0.0, 0.18, 1.335, 0.0, 1.0, 0.0, 0.0])
SHELF_PP_RETRACT = list_to_pose([0.0, 0.55, 1.335, 0.0, 1.0, 0.0, 0.0])

class MasterController(rclpy.node.Node):
    def __init__(self):
        super().__init__("servo_master_node")
        self.move_l_client = self.create_client(Move, "move_l", callback_group=MutuallyExclusiveCallbackGroup())
        self.move_j_client = self.create_client(Move, "move_j", callback_group=MutuallyExclusiveCallbackGroup())
        self.move_relative_client = self.create_client(
            Move, "move_relative", callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.move_to_presaved_pose_client = self.create_client(
            StringTrigger, "move_to_presaved_pose", callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.visual_servo_client = self.create_client(
            VisualServo, "start_visual_servo", callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.force_torque_servo_client = self.create_client(
            ForceTorqueServo, "start_force_torque_servo", callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.save_charuco_pose_client = self.create_client(
            Trigger, "save_charuco_pose", callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.create_service(
            Trigger,
            "run",
            self.run_cb,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.get_logger().info("Master Controller Node has been started.")

    def presaved_pose_move(self, pose):
        req = StringTrigger.Request()
        req.data = pose
        resp = self.move_to_presaved_pose_client.call(req)
        if not resp.success:
            return False
        return True

    def move_linear(self, pose: Pose, move_group: str):
        req = Move.Request()
        req.target_pose = pose
        req.move_group = move_group
        resp = self.move_l_client.call(req)
        if not resp.success:
            return False
        return True

    def move_relative(self, offset: Pose, move_group: str, relative_frame: str):
        req = Move.Request()
        req.target_pose = offset
        req.move_group = move_group
        req.relative_frame = relative_frame
        resp = self.move_relative_client.call(req)
        if not resp.success:
            return False
        return True

    def visual_servo(self, offset):
        req = VisualServo.Request()
        req.offset = offset
        resp = self.visual_servo_client.call(req)
        if not resp.success:
            return False
        return True

    def force_torque_servo(self, theoritical_pick_pose):
        req = ForceTorqueServo.Request()
        req.theoritical_pose = theoritical_pick_pose
        resp = self.force_torque_servo_client.call(req)
        if not resp.success:
            return False
        return True

    def run_cb(self, request, response):
        self.get_logger().info("Run service called.")

        # unstow to start sequence
        self.presaved_pose_move("stow")
        self.presaved_pose_move("home")

        # approach charuco and save pose
        self.move_linear(CHECK_CHARUCO_POSE, "camera_group")
        self.visual_servo(CHARUCO_SERVO_OFFSET)
        self.save_charuco_pose_client.call(Trigger.Request())

        # pick foup from rack
        self.move_relative(RACK_HOME, "foup_group", "saved_charuco_pose")
        self.move_relative(START_FT_SERVO_POSE, "foup_group", "saved_charuco_pose")
        self.force_torque_servo(THEORITICAL_FOUP_PICK_POSE)

        # self.move_relative(THEORITICAL_FOUP_PICK_POSE, "foup_group", "saved_charuco_pose")
        self.move_relative(FOUP_LIFT_UP, "foup_group", "saved_charuco_pose")
        self.move_relative(FOUP_LIFT_UP_RETRACT, "foup_group", "saved_charuco_pose")
        self.presaved_pose_move("home")

        # place foup on shelf
        self.move_linear(SHELF_OUT_POSE, "foup_group")
        self.move_linear(SHELF_IN_POSE, "foup_group")
        time.sleep(2.0)
        self.move_linear(SHELF_PP_CONTACT, "foup_group")
        time.sleep(5.0)
        self.move_linear(SHELF_PP, "foup_group")
        time.sleep(5.0)
        self.move_linear(SHELF_PP_RETRACT, "foup_group")

        # end, show arm
        self.presaved_pose_move("home")
        self.presaved_pose_move("stow")


        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    master_controller_node = MasterController()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
    executor.add_node(master_controller_node)
    executor.spin()
    executor.shutdown()
    master_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
