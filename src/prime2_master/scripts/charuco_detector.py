#!/usr/bin/python3
import rclpy
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import cv2
import numpy as np
from cv_bridge import CvBridge
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
import tf2_geometry_msgs


class CharucoDetector(rclpy.node.Node):
    def __init__(self):
        super().__init__("charuco_detector_node")

        self.aruco_dictionary = "DICT_4X4_1000"
        self.marker_per_row = 9
        self.marker_per_column = 3
        self.checker_size = 0.016
        self.marker_size = 0.012

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.camera_matrix = None
        self.dist_coeffs = None

        self.dictionary = cv2.aruco.getPredefinedDictionary(eval("cv2.aruco." + str(self.aruco_dictionary)))
        self.board = cv2.aruco.CharucoBoard(
            (self.marker_per_row, self.marker_per_column), self.checker_size, self.marker_size, self.dictionary
        )

        self.start_charuco_detection = True
        self.bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()
        self.saved_charuco_pose = None
        self.last_detected_charuco_pose = None

        self.create_service(
            Trigger,
            "save_charuco_pose",
            self.save_charuco_pose_cb,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.charuco_pose_pub = self.create_publisher(
            PoseStamped, "/charuco_pose", 10, callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.detection_image_pub = self.create_publisher(
            Image, "/charuco_detection_image", 10, callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.camera_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.camera_cb, 10, callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/color/camera_info",
            self.camera_info_cb,
            1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.create_timer(0.01, self.timer_cb, callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info("Charuco Detector Node has started.")

    def camera_info_cb(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info("Camera intrinsic parameters received.")
        self.destroy_subscription(self.camera_info_sub)

    def timer_cb(self):
        if self.saved_charuco_pose is not None:
            self.get_logger().info("Publishing saved charuco pose frame.", throttle_duration_sec=10.0)
            self.publish_frame(self.saved_charuco_pose, "base_link", "saved_charuco_pose")

    def camera_cb(self, msg):
        if self.camera_matrix is not None and self.dist_coeffs is not None:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
            if ids is not None:
                retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                    corners, ids, gray, self.board
                )
                if retval > 0:
                    retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                        charuco_corners,
                        charuco_ids,
                        self.board,
                        self.camera_matrix,
                        self.dist_coeffs,
                        np.empty(1),
                        np.empty(1),
                    )
                    if retval:
                        # shift center from btm left corner to center of charuco board
                        charuco_frame_offset = np.array(
                            [
                                self.board.getChessboardSize()[0] * self.board.getSquareLength() / 2,
                                self.board.getChessboardSize()[1] * self.board.getSquareLength() / 2,
                                0,
                            ]
                        )
                        rmat, _ = cv2.Rodrigues(rvec)
                        quat = R.from_matrix(rmat).as_quat()
                        qx, qy, qz, qw = quat
                        tvec += (np.dot(rmat, charuco_frame_offset)).reshape(-1, 1)
                        pose = PoseStamped()
                        pose.header = msg.header
                        pose.pose.position.x = tvec[0, 0]
                        pose.pose.position.y = tvec[1, 0]
                        pose.pose.position.z = tvec[2, 0]
                        pose.pose.orientation.x = qx
                        pose.pose.orientation.y = qy
                        pose.pose.orientation.z = qz
                        pose.pose.orientation.w = qw

                        self.publish_frame(pose, f"camera_color_optical_frame", "charuco_pose_camera")
                        self.charuco_pose_pub.publish(pose)
                        self.last_detected_charuco_pose = pose

                        axis_length = 0.05
                        cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, axis_length)

                        # Overlay pose
                        r = R.from_matrix(rmat)
                        roll, pitch, yaw = r.as_euler("xyz", degrees=True)
                        pos_text = f"Position (m): x={tvec[0,0]:.3f}, y={tvec[1,0]:.3f}, z={tvec[2,0]:.3f}"
                        rpy_text = f"Orientation (deg): roll={roll:.1f}, pitch={pitch:.1f}, yaw={yaw:.1f}"
                        cv2.putText(cv_image, pos_text, (10, 630), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 0), 3)
                        cv2.putText(cv_image, rpy_text, (10, 680), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 0), 3)

            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            image_msg.header = msg.header
            self.detection_image_pub.publish(image_msg)

    def publish_frame(self, pose, parent_frame, child_frame):
        t = TransformStamped()
        if isinstance(pose, PoseStamped):
            t.header.stamp = pose.header.stamp
            p = pose.pose
        elif isinstance(pose, Pose):
            t.header.stamp = self.get_clock().now().to_msg()
            p = pose
        else:
            raise TypeError("pose must be Pose or PoseStamped")

        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = p.position.x
        t.transform.translation.y = p.position.y
        t.transform.translation.z = p.position.z
        t.transform.rotation = p.orientation
        self.tf_broadcaster.sendTransform(t)

    def save_charuco_pose_cb(self, request, response):
        # transform to base_link
        trans = self.tf_buffer.lookup_transform("base_link", f"camera_color_optical_frame", rclpy.time.Time())
        self.saved_charuco_pose = tf2_geometry_msgs.do_transform_pose(self.last_detected_charuco_pose.pose, trans)
        self.get_logger().info("Charuco pose saved.")
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    charuco_detector_node = CharucoDetector()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
    executor.add_node(charuco_detector_node)
    executor.spin()
    executor.shutdown()
    charuco_detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
