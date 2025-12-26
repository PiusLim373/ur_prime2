#ifndef MGI_HPP
#define MGI_HPP

#include <memory>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/time.h>
#include "prime2_master/srv/string_trigger.hpp"
#include "prime2_master/srv/move.hpp"

class MGI : public rclcpp::Node
{
public:
    MGI();

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr foup_group_pose_pub, camera_group_pose_pub;
    rclcpp::Service<prime2_master::srv::StringTrigger>::SharedPtr move_to_presaved_pose_server_;
    moveit::planning_interface::MoveGroupInterfacePtr foup_move_group_, camera_move_group_;
    std::unordered_map<std::string, moveit::planning_interface::MoveGroupInterfacePtr> move_groups_map_;
    rclcpp::Service<prime2_master::srv::Move>::SharedPtr move_l_server_, move_j_server_, move_relative_server_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Executor::SharedPtr executor_;
    std::thread executor_thread_;

    void moveToPresavedPoseCB(const std::shared_ptr<prime2_master::srv::StringTrigger::Request> request,
                              const std::shared_ptr<prime2_master::srv::StringTrigger::Response> response);
    void moveLCB(const std::shared_ptr<prime2_master::srv::Move::Request> request,
                 const std::shared_ptr<prime2_master::srv::Move::Response> response);
    void moveJCB(const std::shared_ptr<prime2_master::srv::Move::Request> request,
                 const std::shared_ptr<prime2_master::srv::Move::Response> response);
    void moveRelativeCB(const std::shared_ptr<prime2_master::srv::Move::Request> request,
                        const std::shared_ptr<prime2_master::srv::Move::Response> response);
    void timerCB();

    bool moveToPose(const geometry_msgs::msg::Pose &target_pose, const std::string &move_group);
    bool moveLinear(const geometry_msgs::msg::Pose &target_pose, const std::string &move_group);
};

#endif