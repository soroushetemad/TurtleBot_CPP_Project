/**
 * @file reach_target_client.cpp
 * @author Soroush Etemad and Harsh Senjaliya 
 * @brief 
 * @version 0.1
 * @date 2024-08-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <map>
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "reach_target_action.hpp"
/**
 * @brief Send goal to action server (if the target has changed)
 * 
 */
void RobotTargetClient::send_goal() {
    auto goal_msg = RobotTarget::Goal();
    if(this->prev_target_x_ != this->next_target_x_ && this->prev_target_y_ != this->next_target_y_){
        goal_msg.target.x = next_target_x_;
        goal_msg.target.y = next_target_y_;
        RCLCPP_INFO_STREAM(this->get_logger(), "Sending goal x[" << goal_msg.target.x << "], y[" << goal_msg.target.y << "]");
        auto goal_options = rclcpp_action::Client<RobotTarget>::SendGoalOptions();
        // goal_options.goal_response_callback = std::bind(&RobotTargetClient::goal_response_callback, this, std::placeholders::_1);
        goal_options.feedback_callback = std::bind(&RobotTargetClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        goal_options.result_callback = std::bind(&RobotTargetClient::result_callback, this, std::placeholders::_1);

        auto goal_handle_future = action_client_->async_send_goal(goal_msg, goal_options);

        this->prev_target_x_ = this->next_target_x_;    
        this->prev_target_y_ = this->next_target_y_;
    }
}
/**
 * @brief Callback function for goal response
 * 
 * @param future The future object containing the goal handle
 */
void RobotTargetClient::goal_response_callback(std::shared_future<GoalHandle::SharedPtr> future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result...");
    }
}

/**
 * @brief Callback function for goal response.
 * 
 * @param future The feedback object containing the distance to the goal.
 */
void RobotTargetClient::feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const RobotTarget::Feedback> feedback) {
    
    RCLCPP_INFO(this->get_logger(), "Received feedback: distance to goal = %f", feedback->distance_to_goal);

    if (feedback->distance_to_goal < 0.04 && this->flag==0) {
        this->flag = 1;
        RCLCPP_INFO_STREAM(this->get_logger(), " CURRENT WAYPOINT---------> " << this->current_waypoint_ );
        next_target_x_ = this->part_map_world_frame_[this->waypoints_[this->current_waypoint_-1]+"_part"].position.x;
        next_target_y_ = this->part_map_world_frame_[this->waypoints_[this->current_waypoint_-1]+"_part"].position.y;
        RCLCPP_INFO_STREAM(this->get_logger(), "target x:" << next_target_x_ << "],target y:" << next_target_y_ << "]");
        this->current_waypoint_++;
    }
    if (feedback->distance_to_goal> 0.04){
        this->flag = 0;
    }
}
/**
 * @brief Callback function for result
 * 
 * @param result The result object containing the result code and message
 */
void RobotTargetClient::result_callback(const GoalHandle::WrappedResult& result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), ">>>>>>>>Goal succeeded: %s", result.result->message.c_str());
    } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
        RCLCPP_ERROR(this->get_logger(), ">>>>>>>>Goal was aborted");
    } else {
        RCLCPP_ERROR(this->get_logger(), ">>>>>>>>Unknown result code");
    }
}

/**
 * @brief Callback function for Camera 1
 * 
 * @param msg The camera image message containing part information.
 */
void RobotTargetClient::camera1_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
    if(!this->camera1_first_message_received_){
        int int_key = msg->part_poses[0].part.color;
        this->key1 = get_battery_type(int_key);
        part_map_[this->key1] = msg->part_poses[0].pose;
        this->camera1_first_message_received_ = true;
    }
}
/**
 * @brief Callback function for Camera 2
 * 
 * @param msg The camera image message containing part information.
 */
void RobotTargetClient::camera2_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
    if(!this->camera2_first_message_received_){
        int int_key = msg->part_poses[0].part.color;
        this->key2 = get_battery_type(int_key);
        part_map_[this->key2] = msg->part_poses[0].pose;
        this->camera2_first_message_received_ = true;

    }
}
/**
 * @brief Callback function for Camera 3
 * 
 * @param msg The camera image message containing part information.
 */
void RobotTargetClient::camera3_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
    if(!this->camera3_first_message_received_){
        int int_key = msg->part_poses[0].part.color;
        this->key3 = get_battery_type(int_key);
        part_map_[this->key3] = msg->part_poses[0].pose;
        this->camera3_first_message_received_ = true;
    }
}
/**
 * @brief Callback function for Camera 4
 * 
 * @param msg The camera image message containing part information.
 */
void RobotTargetClient::camera4_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
    if(!this->camera4_first_message_received_){
        int int_key = msg->part_poses[0].part.color;
        this->key4 = get_battery_type(int_key);
        part_map_[this->key4] = msg->part_poses[0].pose;
        this->camera4_first_message_received_ = true;
    }
}
/**
 * @brief Callback function for Camera 5
 * 
 * @param msg The camera image message containing part information.
 */
void RobotTargetClient::camera5_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
    if(!this->camera5_first_message_received_){
        int int_key = msg->part_poses[0].part.color;
        this->key5 = get_battery_type(int_key);
        part_map_[this->key5] = msg->part_poses[0].pose;
        this->camera5_first_message_received_ = true;
    }
}
/**
 * @brief Determines the battery type based on an integer key.
 * 
 * @param x The integer key representing the battery color.
 * @return The corresponding battery color as a string.
 */
std::string RobotTargetClient::get_battery_type(int x){
    switch (x)
    {
    case 0:
        return "red";
        break;
    case 1:
        return "green";
        break;
    case 2:
        return "blue";
        break;
    case 3:
        return "orange";
        break;
    case 4:
        return "purple";
        break;
    }
}

/**
 * @brief Timer callback function to periodically broadcast dynamic transforms for parts detected by cameras.
 * 
 * This function checks if the first message from each camera has been received. If it has, it creates and sends a 
 * dynamic transform for the part detected by that camera. The transform includes the position and orientation of the part.
 */
void RobotTargetClient::broadcast_timer_cb_()
{
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped;
    // Check if the first message from camera 1 has been received
    if(this->camera1_first_message_received_){
        dynamic_transform_stamped.header.stamp = this->get_clock()->now();
        dynamic_transform_stamped.header.frame_id = "camera1_frame";
        dynamic_transform_stamped.child_frame_id = this->key1 + "_part";
        // Set the translation components of the transform
        dynamic_transform_stamped.transform.translation.x =  part_map_[this->key1].position.x;
        dynamic_transform_stamped.transform.translation.y = part_map_[this->key1].position.y;
        dynamic_transform_stamped.transform.translation.z = part_map_[this->key1].position.z;
        // Set the rotation components of the transform
        dynamic_transform_stamped.transform.rotation.x = part_map_[this->key1].orientation.x;
        dynamic_transform_stamped.transform.rotation.y = part_map_[this->key1].orientation.y;
        dynamic_transform_stamped.transform.rotation.z = part_map_[this->key1].orientation.z;
        dynamic_transform_stamped.transform.rotation.w = part_map_[this->key1].orientation.w;
        tf_broadcaster_->sendTransform(dynamic_transform_stamped); // Send the transform
    }

    if(this->camera2_first_message_received_){
        dynamic_transform_stamped.header.stamp = this->get_clock()->now();
        dynamic_transform_stamped.header.frame_id = "camera2_frame";
        dynamic_transform_stamped.child_frame_id = this->key2 + "_part";

        dynamic_transform_stamped.transform.translation.x = part_map_[this->key2].position.x;
        dynamic_transform_stamped.transform.translation.y = part_map_[this->key2].position.y;
        dynamic_transform_stamped.transform.translation.z = part_map_[this->key2].position.z;

        dynamic_transform_stamped.transform.rotation.x = part_map_[this->key2].orientation.x;
        dynamic_transform_stamped.transform.rotation.y = part_map_[this->key2].orientation.y;
        dynamic_transform_stamped.transform.rotation.z = part_map_[this->key2].orientation.z;
        dynamic_transform_stamped.transform.rotation.w = part_map_[this->key2].orientation.w;
        tf_broadcaster_->sendTransform(dynamic_transform_stamped);
    }

    if(this->camera3_first_message_received_){
        dynamic_transform_stamped.header.stamp = this->get_clock()->now();
        dynamic_transform_stamped.header.frame_id = "camera3_frame";
        dynamic_transform_stamped.child_frame_id = this->key3 + "_part";

        dynamic_transform_stamped.transform.translation.x = part_map_[this->key3].position.x;
        dynamic_transform_stamped.transform.translation.y = part_map_[this->key3].position.y;
        dynamic_transform_stamped.transform.translation.z = part_map_[this->key3].position.z;

        dynamic_transform_stamped.transform.rotation.x = part_map_[this->key3].orientation.x;
        dynamic_transform_stamped.transform.rotation.y = part_map_[this->key3].orientation.y;
        dynamic_transform_stamped.transform.rotation.z = part_map_[this->key3].orientation.z;
        dynamic_transform_stamped.transform.rotation.w = part_map_[this->key3].orientation.w;
        tf_broadcaster_->sendTransform(dynamic_transform_stamped);
    }

    if(this->camera4_first_message_received_){
        dynamic_transform_stamped.header.stamp = this->get_clock()->now();
        dynamic_transform_stamped.header.frame_id = "camera4_frame";
        dynamic_transform_stamped.child_frame_id = this->key4 + "_part";

        dynamic_transform_stamped.transform.translation.x = part_map_[this->key4].position.x;
        dynamic_transform_stamped.transform.translation.y = part_map_[this->key4].position.y;
        dynamic_transform_stamped.transform.translation.z = part_map_[this->key4].position.z;

        dynamic_transform_stamped.transform.rotation.x = part_map_[this->key4].orientation.x;
        dynamic_transform_stamped.transform.rotation.y = part_map_[this->key4].orientation.y;
        dynamic_transform_stamped.transform.rotation.z = part_map_[this->key4].orientation.z;
        dynamic_transform_stamped.transform.rotation.w = part_map_[this->key4].orientation.w;
        tf_broadcaster_->sendTransform(dynamic_transform_stamped);
    }

    if(this->camera5_first_message_received_){
        dynamic_transform_stamped.header.stamp = this->get_clock()->now();
        dynamic_transform_stamped.header.frame_id = "camera5_frame";
        dynamic_transform_stamped.child_frame_id = this->key5 + "_part";

        dynamic_transform_stamped.transform.translation.x = part_map_[this->key5].position.x;
        dynamic_transform_stamped.transform.translation.y = part_map_[this->key5].position.y;
        dynamic_transform_stamped.transform.translation.z = part_map_[this->key5].position.z;

        dynamic_transform_stamped.transform.rotation.x = part_map_[this->key5].orientation.x;
        dynamic_transform_stamped.transform.rotation.y = part_map_[this->key5].orientation.y;
        dynamic_transform_stamped.transform.rotation.z = part_map_[this->key5].orientation.z;
        dynamic_transform_stamped.transform.rotation.w = part_map_[this->key5].orientation.w;
        tf_broadcaster_->sendTransform(dynamic_transform_stamped);
    }
}
/**
 * @brief Retrieves the transform between the source and target frames and stores it in the part map.
 * 
 * This function looks up the transform between the specified source and target frames using the TF buffer. 
 * If the transform is successfully retrieved, it is stored in the part map with the target frame as the key.
 * 
 * @param source_frame The source frame from which the transform originates.
 * @param target_frame The target frame to which the transform is applied.
 */
void RobotTargetClient::listen_transform(const std::string &source_frame, const std::string &target_frame)
{
    geometry_msgs::msg::TransformStamped t_stamped;
    geometry_msgs::msg::Pose pose_out;
    try
    {
        // Try to lookup the transform from source_frame to target_frame
        t_stamped = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 50ms);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
        return;
    }
    // Populate the position fields of the pose object with the translation from the transform
    pose_out.position.x = t_stamped.transform.translation.x;
    pose_out.position.y = t_stamped.transform.translation.y;
    pose_out.position.z = t_stamped.transform.translation.z;
    pose_out.orientation = t_stamped.transform.rotation;

    this->part_map_world_frame_[target_frame] = pose_out;
}
/**
 * @brief Timer callback function to periodically listen for transforms of parts.
 * 
 * This function calls the listen_transform function for each part color to update their transforms in the part map.
 */
void RobotTargetClient::listen_timer_cb_()
{
    listen_transform("world", "red_part");
    listen_transform("world", "blue_part");
    listen_transform("world", "orange_part");
    listen_transform("world", "purple_part");
    listen_transform("world", "green_part");
}
/**
 * @brief Main function to initialize the robot_target_client node and start the executor.
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotTargetClient>("robot_target_client"); // Create a shared pointer to the RobotTargetClient node
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();  // This will start the execution
    rclcpp::shutdown();
}

