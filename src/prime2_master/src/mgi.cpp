#include "prime2_master/mgi.hpp"

MGI::MGI() : Node("mgi_node"),
             node_(std::make_shared<rclcpp::Node>("mgi_spinner_node")),
             executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
{

    move_to_presaved_pose_server_ = this->create_service<prime2_master::srv::StringTrigger>(
        "move_to_presaved_pose",
        std::bind(&MGI::moveToPresavedPoseCB, this, std::placeholders::_1, std::placeholders::_2));

    move_l_server_ = this->create_service<prime2_master::srv::Move>(
        "move_l",
        std::bind(&MGI::moveLCB, this, std::placeholders::_1, std::placeholders::_2));

    move_j_server_ = this->create_service<prime2_master::srv::Move>(
        "move_j",
        std::bind(&MGI::moveJCB, this, std::placeholders::_1, std::placeholders::_2));

    move_relative_server_ = this->create_service<prime2_master::srv::Move>(
        "move_relative",
        std::bind(&MGI::moveRelativeCB, this, std::placeholders::_1, std::placeholders::_2));

    foup_group_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("foup_group_pose", 1);
    camera_group_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("camera_group_pose", 1);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer = this->create_wall_timer(std::chrono::milliseconds(100),
                                    std::bind(&MGI::timerCB, this));

    foup_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "foup_group");
    camera_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "camera_group");
    foup_move_group_->setMaxVelocityScalingFactor(0.3);
    foup_move_group_->setMaxAccelerationScalingFactor(0.3);

    move_groups_map_["foup_group"] = foup_move_group_;
    move_groups_map_["camera_group"] = camera_move_group_;

    executor_->add_node(node_);
    executor_thread_ = std::thread([this]()
                                   { this->executor_->spin(); });
    RCLCPP_INFO(this->get_logger(), "MoveGroupInterface for 'foup_group' has been initialized.");
}

void MGI::timerCB()
{
    foup_group_pose_pub->publish(foup_move_group_->getCurrentPose());
    camera_group_pose_pub->publish(camera_move_group_->getCurrentPose());
}

void MGI::moveToPresavedPoseCB(const std::shared_ptr<prime2_master::srv::StringTrigger::Request> request,
                               const std::shared_ptr<prime2_master::srv::StringTrigger::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Moving to presaved pose: %s", request->data.c_str());
    foup_move_group_->setNamedTarget(request->data);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (foup_move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
        auto result = foup_move_group_->execute(plan);
        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Move to presaved pose '%s' successful", request->data.c_str());
            response->success = true;
            return;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Move to presaved pose '%s' unsuccessful", request->data.c_str());
            response->success = false;
            return;
        }
    }
}

void MGI::moveLCB(const std::shared_ptr<prime2_master::srv::Move::Request> request,
                  const std::shared_ptr<prime2_master::srv::Move::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "MoveL request received for move group: %s", request->move_group.c_str());
    response->success = moveLinear(request->target_pose, request->move_group);
    return;
}

void MGI::moveJCB(const std::shared_ptr<prime2_master::srv::Move::Request> request,
                  const std::shared_ptr<prime2_master::srv::Move::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "MoveJ request received for move group: %s", request->move_group.c_str());
    response->success = moveToPose(request->target_pose, request->move_group);
    return;
}

void MGI::moveRelativeCB(const std::shared_ptr<prime2_master::srv::Move::Request> request,
                         const std::shared_ptr<prime2_master::srv::Move::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "MoveRelative request received");

    geometry_msgs::msg::PoseStamped offset_pose_stamped;
    offset_pose_stamped.header.frame_id = request->relative_frame;
    offset_pose_stamped.pose = request->target_pose;

    geometry_msgs::msg::PoseStamped transformed_target;
    tf_buffer_->transform(offset_pose_stamped, transformed_target, "world", tf2::durationFromSec(0.2));
    response->success = moveLinear(transformed_target.pose, request->move_group);
    return;
}

bool MGI::moveToPose(const geometry_msgs::msg::Pose &target_pose, const std::string &move_group)
{
    auto current_move_group = move_groups_map_[move_group];

    // get ik solutions
    auto robot_model = current_move_group->getRobotModel();
    moveit::core::RobotState state(robot_model);

    // Seed with current state
    state.setToDefaultValues();
    state.setJointGroupPositions(
        move_group,
        current_move_group->getCurrentJointValues());

    const moveit::core::JointModelGroup *jmg =
        robot_model->getJointModelGroup(move_group);

    bool found_ik = state.setFromIK(
        jmg,
        target_pose,
        current_move_group->getEndEffectorLink(),
        0.05);

    if (!found_ik)
    {
        RCLCPP_ERROR(this->get_logger(), "IK failed");
        return false;
    }

    // Inspect joint solution BEFORE execution
    std::vector<double> ik_solution, current_joints;
    state.copyJointGroupPositions(jmg, ik_solution);

    RCLCPP_INFO(this->get_logger(), "IK solution size: %d", ik_solution.size());
    RCLCPP_INFO(this->get_logger(), "shoulder_pan = %.3f rad", ik_solution[0]);
    RCLCPP_INFO(this->get_logger(), "shoulder_lift = %.3f rad", ik_solution[1]);
    RCLCPP_INFO(this->get_logger(), "elbow_lift = %.3f rad", ik_solution[2]);
    RCLCPP_INFO(this->get_logger(), "wrist1 = %.3f rad", ik_solution[3]);
    RCLCPP_INFO(this->get_logger(), "wrist2 = %.3f rad", ik_solution[4]);
    RCLCPP_INFO(this->get_logger(), "wrist3 = %.3f rad", ik_solution[5]);
    RCLCPP_INFO(this->get_logger(), "=================== ");

    foup_move_group_->getCurrentState()->copyJointGroupPositions(jmg, current_joints);
    RCLCPP_INFO(this->get_logger(), "IK solution size: %d", current_joints.size());
    RCLCPP_INFO(this->get_logger(), "shoulder_pan = %.3f rad", current_joints[0]);
    RCLCPP_INFO(this->get_logger(), "shoulder_lift = %.3f rad", current_joints[1]);
    RCLCPP_INFO(this->get_logger(), "elbow_lift = %.3f rad", current_joints[2]);
    RCLCPP_INFO(this->get_logger(), "wrist1 = %.3f rad", current_joints[3]);
    RCLCPP_INFO(this->get_logger(), "wrist2 = %.3f rad", current_joints[4]);
    RCLCPP_INFO(this->get_logger(), "wrist3 = %.3f rad", current_joints[5]);
    RCLCPP_INFO(this->get_logger(), "****************");

    if (abs(current_joints[0]) - abs(ik_solution[0]) > 1.0)
    {
        RCLCPP_ERROR(this->get_logger(), "Large jump in shoulder_pan detected, aborting move");
        return false;
    }

    current_move_group->setJointValueTarget(ik_solution);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (current_move_group->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        return false;
    current_move_group->execute(plan);
    current_move_group->clearPoseTargets();
    return true;
}

bool MGI::moveLinear(const geometry_msgs::msg::Pose &target_pose, const std::string &move_group)
{
    auto current_move_group = move_groups_map_[move_group];

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(current_move_group->getCurrentPose().pose);
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;

    const double eef_step = 0.005;                            // 5 mm
    const double jump_thresh = 2.0;                           // disable jump detection
    current_move_group->setMaxVelocityScalingFactor(0.2);     // 20% of max speed
    current_move_group->setMaxAccelerationScalingFactor(0.2); // 20% of max acceleration

    double fraction = current_move_group->computeCartesianPath(
        waypoints, eef_step, jump_thresh, trajectory);

    if (fraction < 0.99)
    {
        RCLCPP_ERROR(this->get_logger(), "Cartesian path failed (%.2f%% achieved)", fraction * 100.0);
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Cartesian path computed (%.2f%% achieved)", fraction * 100.0);
    RCLCPP_INFO(this->get_logger(), "trajectory has %zu points", trajectory.joint_trajectory.points.size());

    // Rescale timing
    robot_trajectory::RobotTrajectory rt(current_move_group->getCurrentState()->getRobotModel(), move_group);
    rt.setRobotTrajectoryMsg(*current_move_group->getCurrentState(), trajectory);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(rt, 0.15, 0.15); // velocity_scale, acceleration_scale

    rt.getRobotTrajectoryMsg(trajectory);

    RCLCPP_INFO(this->get_logger(), "After interpolation, trajectory has %zu points", trajectory.joint_trajectory.points.size());

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    auto result = current_move_group->execute(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Move Linear execution successful");
        return true;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("move_group"), "Trajectory execution failed");
        return false;
    }
}
