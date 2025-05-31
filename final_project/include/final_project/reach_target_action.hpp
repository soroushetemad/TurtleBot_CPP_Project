/**
 * @file reach_target_action.hpp
 * @author Soroush Etemad and Harsh Senjaliya 
 * @brief 
 * @version 0.1
 * @date 2024-08-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "geometry_msgs/msg/twist.hpp"
#include "mage_msgs/action/robot_target.hpp"
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <map>
#include <array>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

using namespace std::chrono_literals;
using RobotTarget = mage_msgs::action::RobotTarget;
using GoalHandle = rclcpp_action::ClientGoalHandle<RobotTarget>;


/**
 * @class RobotTargetClient
 * @brief A class that handles robot target actions and part transforms.
 * 
 * This class is responsible for subscribing to camera topics, receiving part transforms,
 * broadcasting transforms, and managing robot target actions.
 */
class RobotTargetClient : public rclcpp::Node {

   public:
    // Declaring waypoint variables
    std::string waypoint1;
    std::string waypoint2;
    std::string waypoint3;
    std::string waypoint4;
    std::string waypoint5;
    std::map<std::string, geometry_msgs::msg::Pose> part_map_;
    std::map<std::string, geometry_msgs::msg::Pose> part_map_world_frame_;
/**
 * @brief Construct a new Robot Target Client object
 * 
 * Initializes the node and sets up callback groups, subscribers, and action clients.
 * 
 * @param node_name The name of the node
 */
    explicit RobotTargetClient(const std::string& node_name)
        : Node(node_name), next_target_x_{0.1}, next_target_y_{-0.1}, camera1_first_message_received_(false),
        camera2_first_message_received_(false), camera3_first_message_received_(false), camera4_first_message_received_(false),
        camera5_first_message_received_(false){

        // Create a mutually exclusive callback group
        group1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Create a mutually exclusive callback group
        rclcpp::SubscriptionOptions sub_option;
        group2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        sub_option.callback_group = group2_;

        // Create a callback group for the action client
        action_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        
        // Create action client
        action_client_ = rclcpp_action::create_client<RobotTarget>(this, "reach_target", action_group_);

        // Initialize publisher and subscriber
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Connect to action server
        timer_ = this->create_wall_timer(1s, std::bind(&RobotTargetClient::send_goal, this), group1_);

        // Initialize camera subscribers
        camera1_sub_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "/mage/camera1/image", rclcpp::SensorDataQoS(),
            std::bind(&RobotTargetClient::camera1_callback, this, std::placeholders::_1), sub_option);
        camera2_sub_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "/mage/camera2/image", rclcpp::SensorDataQoS(),
            std::bind(&RobotTargetClient::camera2_callback, this, std::placeholders::_1), sub_option);
        camera3_sub_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "/mage/camera3/image", rclcpp::SensorDataQoS(),
            std::bind(&RobotTargetClient::camera3_callback, this, std::placeholders::_1), sub_option);
        camera4_sub_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "/mage/camera4/image", rclcpp::SensorDataQoS(),
            std::bind(&RobotTargetClient::camera4_callback, this, std::placeholders::_1), sub_option);
        camera5_sub_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "/mage/camera5/image", rclcpp::SensorDataQoS(),
            std::bind(&RobotTargetClient::camera5_callback, this, std::placeholders::_1), sub_option);
        
        // Declaring parameters for batteries:
        waypoint1 = this->declare_parameter("waypoint1", "green");
        waypoint2 = this->declare_parameter("waypoint2", "red");
        waypoint3 = this->declare_parameter("waypoint3", "orange");
        waypoint4 = this->declare_parameter("waypoint4", "purple");
        waypoint5 = this->declare_parameter("waypoint5", "blue");

        // Retrieve parameters for waypoints
        waypoint1 = this->get_parameter("waypoint1").as_string();
        waypoint2 = this->get_parameter("waypoint2").as_string();
        waypoint3 = this->get_parameter("waypoint3").as_string();
        waypoint4 = this->get_parameter("waypoint4").as_string();        
        waypoint5 = this->get_parameter("waypoint5").as_string();

        // Initialize maps with waypoints and empty poses
        part_map_[waypoint1] = geometry_msgs::msg::Pose();
        part_map_[waypoint2] = geometry_msgs::msg::Pose();
        part_map_[waypoint3] = geometry_msgs::msg::Pose();
        part_map_[waypoint4] = geometry_msgs::msg::Pose();
        part_map_[waypoint5] = geometry_msgs::msg::Pose();

        // Initialize the pose for each waypoint in the world frame map and concatinate _part for the frame name
        part_map_world_frame_[waypoint1+"_part"] = geometry_msgs::msg::Pose();
        part_map_world_frame_[waypoint2+"_part"] = geometry_msgs::msg::Pose();
        part_map_world_frame_[waypoint3+"_part"] = geometry_msgs::msg::Pose();
        part_map_world_frame_[waypoint4+"_part"] = geometry_msgs::msg::Pose();
        part_map_world_frame_[waypoint5+"_part"] = geometry_msgs::msg::Pose();

        // Populate the list of waypoints
        waypoints_.push_back(waypoint1);
        waypoints_.push_back(waypoint2);
        waypoints_.push_back(waypoint3);
        waypoints_.push_back(waypoint4);
        waypoints_.push_back(waypoint5);

        current_waypoint_ = 1;
        prev_target_x_ = 0.0;
        prev_target_y_ = 0.0;   

        // Create a transform broadcaster and initialize its timer
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        // timer to publish the transform
        broadcast_timer_ = this->create_wall_timer(100ms, std::bind(&RobotTargetClient::broadcast_timer_cb_, this));

        // load a buffer of transforms
        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // timer to listen to the transforms
        listen_timer_ = this->create_wall_timer(1s, std::bind(&RobotTargetClient::listen_timer_cb_, this));
    }

   private:
   /**
    * @brief Sends a goal to the action server
    * 
    */
    void send_goal();
    /**
     * @brief Callback for receiving goal response from the action server
     * 
     * @param future A future that will be fulfilled when the goal response is received
     */
    void goal_response_callback(std::shared_future<GoalHandle::SharedPtr> future);
     /**
     * @brief Callback for receiving feedback from the action server
     * 
     * @param goal_handle Handle to the goal
     * @param feedback Feedback from the action server
     */
    void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const RobotTarget::Feedback> feedback);
    /**
     * @brief Callback for receiving the result from the action server
     * 
     * @param result The result of the action
     */ 
    void result_callback(const GoalHandle::WrappedResult& result);
    /**
     * @brief Callback for processing camera1 image messages
     * 
     * @param msg The image message from camera1
     */
    void camera1_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    /**
     * @brief Callback for processing camera2 image messages
     * 
     * @param msg The image message from camera2
     */
    void camera2_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    /**
     * @brief Callback for processing camera3 image messages
     * 
     * @param msg The image message from camera3
     */
    void camera3_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    /**
     * @brief Callback for processing camera4 image messages
     * 
     * @param msg The image message from camera4
     */
    void camera4_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    /**
     * @brief Callback for processing camera5 image messages
     * 
     * @param msg The image message from camera5
     */
    void camera5_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /** 
     * @brief Timer callback function to periodically broadcast dynamic transforms for parts detected by cameras.
     * 
     * This function checks if the first message from each camera has been received. If it has, it creates and sends a 
     * dynamic transform for the part detected by that camera. The transform includes the position and orientation of the part.
     */
    void broadcast_timer_cb_();

    /**
     * @brief Listen to a transform
     *
     * @param source_frame Source frame (child frame) of the transform
     * @param target_frame Target frame (parent frame) of the transform
     */
    void listen_transform(const std::string &source_frame, const std::string &target_frame);

    /**
     * @brief Timer to listen to the transform
     *
     */
    void listen_timer_cb_();
    
   private:
    rclcpp::CallbackGroup::SharedPtr action_group_; // Callback group for action client
    rclcpp::TimerBase::SharedPtr timer_; ///< Timer for sending goals
    rclcpp_action::Client<RobotTarget>::SharedPtr action_client_; // Action client for RobotTarget action
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_; // Publisher for cmd_vel topic
    rclcpp::CallbackGroup::SharedPtr group1_; // Callback group for general callbacks
    rclcpp::CallbackGroup::SharedPtr group2_; // Callback group for subscription options

    // Broadcaster object 
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // Wall timer object for the broadcaster
    rclcpp::TimerBase::SharedPtr broadcast_timer_;

    // Subscriber declarations
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera1_sub_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera2_sub_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera3_sub_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera4_sub_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera5_sub_;
    
    // Flags indicating if the first message from each camera has been received
    bool camera1_first_message_received_;
    bool camera2_first_message_received_;
    bool camera3_first_message_received_;
    bool camera4_first_message_received_;
    bool camera5_first_message_received_;
    // Boolean variable to store the value of the parameter "listen"
    bool param_listen_; 

    // Initialize next and previous target coordinates (x,y)
    double next_target_x_;
    double next_target_y_;
    double prev_target_x_;
    double prev_target_y_;
    // Initialize keys 
    std::string key1;
    std::string key2;
    std::string key3;
    std::string key4;
    std::string key5;
    // Gets the type of battery based on an index
    std::string get_battery_type(int);

    int current_waypoint_;
    int flag = 0;
    std::vector<std::string> waypoints_;

    // Buffer that stores several seconds of transforms for easy lookup by the listener.
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    // Transform listener object 
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    // Wall timer object 
    rclcpp::TimerBase::SharedPtr listen_timer_;


};