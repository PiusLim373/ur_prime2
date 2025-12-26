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
import tf2_geometry_msgs
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class PIDController:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, output_limit=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_limit = output_limit

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

    def compute(self, error):
        now = time.time()
        dt = 0.0
        if self.prev_time is not None:
            dt = now - self.prev_time

        # Proportional
        P = self.Kp * error

        # Integral
        if dt > 0:
            self.integral += error * dt
        I = self.Ki * self.integral

        # Derivative
        D = 0.0
        if dt > 0:
            D = self.Kd * (error - self.prev_error) / dt

        # PID output
        output = P + I + D

        # Clamp output
        if self.output_limit is not None:
            output = max(min(output, self.output_limit), -self.output_limit)

        # Save for next iteration
        self.prev_error = error
        self.prev_time = now

        return output


class ServoMaster(rclpy.node.Node):
    def __init__(self):
        super().__init__("servo_master_node")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_service(
            VisualServo,
            "start_visual_servo",
            self.start_visual_servo_cb,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.create_service(
            ForceTorqueServo,
            "start_force_torque_servo",
            self.start_force_torque_servo_cb,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.robot_pose = PoseStamped()
        self.servo_type = ""
        self.servo_state = ""
        self.visual_servo_offset = []
        self.force_torque_servo_target = Pose()

        self.start_servo_client = self.create_client(
            Trigger, "/servo_node/start_servo", callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.stop_servo_client = self.create_client(
            Trigger, "/servo_node/stop_servo", callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.twist_joy_pub = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10, callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.charuco_pose_sub = self.create_subscription(
            PoseStamped, "/charuco_pose", self.charuco_pose_cb, 10, callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.wrench_sub = self.create_subscription(
            WrenchStamped, "/wrench", self.wrench_cb, 10, callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.robot_pose_sub = self.create_subscription(
            PoseStamped, "/foup_group_pose", self.robot_pose_cb, 10, callback_group=MutuallyExclusiveCallbackGroup()
        )

        # Linear PID for X, Y, Z
        self.pid_x = PIDController(Kp=10.0, Ki=-0.0, Kd=0.1, output_limit=1.0)
        self.pid_y = PIDController(Kp=10.0, Ki=-0.0, Kd=0.1, output_limit=1.0)
        self.pid_z = PIDController(Kp=10.0, Ki=-0.0, Kd=0.1, output_limit=1.0)

        # Angular PID for roll, pitch, yaw
        self.pid_roll = PIDController(Kp=10.0, Ki=-0.0, Kd=0.1, output_limit=1.0)
        self.pid_pitch = PIDController(Kp=10.0, Ki=-0.0, Kd=0.1, output_limit=1.0)
        self.pid_yaw = PIDController(Kp=10.0, Ki=-0.0, Kd=0.1, output_limit=1.0)

        self.wrench = Wrench()

        self.timer = self.create_timer(
            0.01, self.force_torque_servo_cycle, callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.get_logger().info("Servo Master Node has started.")

    def force_torque_servo_cycle(self):
        if self.servo_type != "force_torque":
            return

        data = TwistStamped()
        data.header.stamp = self.get_clock().now().to_msg()
        data.header.frame_id = "foup_tcp"
        data.twist.linear.y = -0.4

        dist_to_target = (
            (self.robot_pose.pose.position.x - self.force_torque_servo_target.position.x) ** 2
            + (self.robot_pose.pose.position.y - self.force_torque_servo_target.position.y) ** 2
            + (self.robot_pose.pose.position.z - self.force_torque_servo_target.position.z) ** 2
        ) ** 0.5
        if self.wrench.force.y > 5.0:
            if dist_to_target < 0.01:
                self.get_logger().info(
                    "Foup fully inside the holder, current y force: {:.3f} N".format(self.wrench.force.y)
                )
                self.servo_state = "completed"
                self.servo_type = ""
                return

            self.get_logger().info(f"Foup contact detected")
            self.get_logger().info(
                f"force x: {self.wrench.force.x:.3f} N, force y: {self.wrench.force.y:.3f} N, force z: {self.wrench.force.z:.3f} N"
            )
            self.get_logger().info(
                f"torque x: {self.wrench.torque.x:.3f} Nm, torque y: {self.wrench.torque.y:.3f} Nm, torque z: {self.wrench.torque.z:.3f} Nm"
            )
            data.twist.angular.z = self.wrench.torque.z * 5.0
            data.twist.linear.x = self.wrench.force.x * 5.0
            data.twist.linear.y = 0.5
            self.get_logger().info(
                f"Applying yaw correction: {data.twist.angular.z:.3f} rad/s, linear x correction: {data.twist.linear.x:.3f} m/s"
            )

        if self.wrench.force.y > 500.0:
            self.get_logger().info(
                "excessive force detected, stopping, current y force: {:.3f} N".format(self.wrench.force.y)
            )
            self.servo_state = "error"
            self.servo_type = ""
            return

        self.twist_joy_pub.publish(data)
        self.get_logger().info(
            f"Published Twist Command Linear: x: {data.twist.linear.x}, y: {data.twist.linear.y}, z: {data.twist.linear.z}",
            throttle_duration_sec=2.0,
        )
        self.get_logger().info(
            f"Published Twist Command Angular: x: {data.twist.angular.x}, y: {data.twist.angular.y}, z: {data.twist.angular.z}",
            throttle_duration_sec=2.0,
        )
        self.get_logger().info(
            f"Normalised Theoretical Distance to Target: {dist_to_target:.4f} m", throttle_duration_sec=2.0
        )

    def charuco_pose_cb(self, msg):
        if self.servo_type != "charuco":
            return

        quat = msg.pose.orientation
        r = R.from_quat([quat.x, quat.y, quat.z, quat.w])
        roll, pitch, yaw = r.as_euler("xyz", degrees=False)

        offset_x = self.visual_servo_offset[0] - msg.pose.position.x
        offset_y = self.visual_servo_offset[1] - msg.pose.position.y
        offset_z = self.visual_servo_offset[2] - msg.pose.position.z
        offset_roll = self.visual_servo_offset[3] - roll
        offset_pitch = self.visual_servo_offset[4] - pitch
        offset_yaw = self.visual_servo_offset[5] - yaw

        pos_norm = (offset_x**2 + offset_y**2 + offset_z**2) ** 0.5
        rot_norm = (offset_roll**2 + offset_pitch**2 + offset_yaw**2) ** 0.5

        if pos_norm < 0.01 and rot_norm < 0.01:
            self.get_logger().info("Charuco visual servoing completed.")
            self.servo_state = "completed"
            self.servo_type = ""
            return

        vx = self.pid_x.compute(offset_x)
        vy = self.pid_y.compute(offset_y)
        vz = self.pid_z.compute(offset_z)
        wx = self.pid_roll.compute(offset_roll)
        wy = self.pid_pitch.compute(offset_pitch)
        wz = self.pid_yaw.compute(offset_yaw)

        data = TwistStamped()
        data.header.stamp = self.get_clock().now().to_msg()
        data.header.frame_id = "camera_tcp"
        data.twist.linear.x = -vx
        data.twist.linear.y = -vy
        data.twist.linear.z = -vz
        data.twist.angular.x = -wx
        data.twist.angular.y = -wy
        data.twist.angular.z = -wz
        self.twist_joy_pub.publish(data)
        self.get_logger().info(
            f"Published Twist Command Linear: x: {data.twist.linear.x}, y: {data.twist.linear.y}, z: {data.twist.linear.z}",
            throttle_duration_sec=2.0,
        )
        self.get_logger().info(
            f"Published Twist Command Angular: x: {data.twist.angular.x}, y: {data.twist.angular.y}, z: {data.twist.angular.z}",
            throttle_duration_sec=2.0,
        )
        self.get_logger().info(f"Pos Error: {pos_norm:.4f}, Rot Error: {rot_norm:.4f}", throttle_duration_sec=2.0)

    def wrench_cb(self, msg: WrenchStamped):
        self.wrench = msg.wrench

    def robot_pose_cb(self, msg):
        self.robot_pose = msg

    def start_visual_servo_cb(self, request, response):
        self.get_logger().info("Starting visual servoing...")
        req = Trigger.Request()
        resp = self.start_servo_client.call(req)
        if not resp.success:
            self.get_logger().info("Failed to start servo, stopping everything.")
            response.success = False
            return response
        self.visual_servo_offset = request.offset  # [x, y, z, roll, pitch, yaw]
        self.servo_state = "started"
        self.servo_type = "charuco"
        while self.servo_state != "completed":
            time.sleep(1.0)

        req = Trigger.Request()
        resp = self.stop_servo_client.call(req)
        if not resp.success:
            self.get_logger().info("Failed to stop servo, stopping everything.")
            response.success = False
            return response

        response.success = True
        return response

    def start_force_torque_servo_cb(self, request, response):
        self.get_logger().info("Starting force torque servoing...")
        req = Trigger.Request()
        resp = self.start_servo_client.call(req)
        if not resp.success:
            self.get_logger().info("Failed to start servo, stopping everything.")
            response.success = False
            return response

        trans = self.tf_buffer.lookup_transform("world", "saved_charuco_pose", rclpy.time.Time())
        self.force_torque_servo_target = tf2_geometry_msgs.do_transform_pose(request.theoritical_pose, trans)
        self.servo_state = "started"
        self.servo_type = "force_torque"
        while self.servo_state != "completed" and self.servo_state != "error":
            time.sleep(1.0)

        req = Trigger.Request()
        resp = self.stop_servo_client.call(req)
        if not resp.success:
            self.get_logger().info("Failed to stop servo, stopping everything.")
            response.success = False
            return response

        response.success = True if self.servo_state == "completed" else False
        return response


def main(args=None):
    rclpy.init(args=args)
    servo_master_node = ServoMaster()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
    executor.add_node(servo_master_node)
    executor.spin()
    executor.shutdown()
    servo_master_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
