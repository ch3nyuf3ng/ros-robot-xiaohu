#pragma once

#ifndef XIAOHU_MOVE_OBJECT_NODE_HPP
#define XIAOHU_MOVE_OBJECT_NODE_HPP
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include "wpb_home_behaviors/Coord.h"
#include "xiaohu/MoveTaskMsg.h"
#include "xiaohu/foundation/manipulator_control.hpp"
#include "xiaohu/foundation/measurement.hpp"
#include <cstddef>
#include <deque>
#include <string>

namespace xiaohu_robot {
namespace move_objects_node {
class StateMachine final {
public:
    StateMachine(std::string base_name);
    void run();

private:
    enum class TaskState {
        READY,
        GO_TO_STORAGE_PLACE,
        OBJECT_DETECT,
        GRAB_OBJECT,
        GO_TO_DROP_PLACE,
        PUT_DOWN_OBJECT,
        STEP_BACKWARD,
        RETRACT_MANIPULATOR,
        DONE,
        GO_TO_BASE
    };

    enum class Behavior { START_OBJECT_DETECTION, STOP_OBJECT_DETECTION };

    struct MoveTask {
        std::string storage_place_name;
        std::string drop_place_name;
    };

    using string_msg_ptr = std_msgs::String::ConstPtr const&;
    using move_task_msg_ptr = xiaohu::MoveTaskMsg::ConstPtr const&;
    using coordinates_msg_ptr = wpb_home_behaviors::CoordConstPtr const&;
    using velocity_msg = geometry_msgs::Twist;
    using string_msg = std_msgs::String;
    using position_msg = geometry_msgs::Pose;
    using manipulator_control_msg = sensor_msgs::JointState;
    using node_handle_t = ros::NodeHandle;
    using move_task_queue = std::deque<MoveTask>;
    using duration = Measurement<UnitDuration>;
    using string = std::string;

    duration timing;
    TaskState task_state;
    node_handle_t node_handle;
    move_task_queue move_tasks;
    const std::string base_name;
    const ros::Publisher velocity_publisher;
    const ros::Publisher behaviors_publisher;
    const ros::Publisher waypoint_publisher;
    const ros::Publisher grab_object_publisher;
    const ros::Publisher manipulatior_control_publisher;
    const ros::Subscriber object_coordinate_subscriber;
    const ros::Subscriber navigation_result_subscriber;
    const ros::Subscriber grab_result_subscriber;
    const ros::Subscriber move_task_subscriber;
    const ros::Subscriber cancellation_subscriber;
    static constexpr double loops_per_second{10};

    MoveTask get_current_move_task() const;
    TaskState get_current_task_state() const;
    Measurement<UnitDuration> get_timing() const;
    void set_next_task_state(TaskState next_state);
    void increment_timing();
    void reset_timing();

    void go_to_base();
    void go_to_storage();
    void put_down_object();
    void step_backward();
    void retract_manipulaor();
    void show_tasks() const;

    void process_object_detect_message(coordinates_msg_ptr coordinates_ptr);
    void process_navigation_result_message(string_msg_ptr message_ptr);
    void process_grab_result_message(string_msg_ptr message_ptr);
    void process_move_task_message(move_task_msg_ptr message_ptr);
    void process_cancel_task_message(string_msg_ptr message_ptr);

    void notify_control_robot_speed(double axis_x_linear_speed);
    void notify_change_robot_behavior(Behavior behavior);
    void notify_navigate_robot(string waypoint_name);
    void notify_control_robot_manipulator(ManipulatorControl const& plan);
    void notify_grab_object(coordinates_msg_ptr coordinates_ptr, size_t index);

    static velocity_msg create_velocity_command(double axis_x_linear_speed);
    static string_msg create_waypoint_message(std::string waypoint_name);
    static string_msg create_behavior_message(Behavior behavior);
    static position_msg create_position_message(double x, double y, double z);
    static void display_detected_objects(coordinates_msg_ptr coordinates_ptr);
};
}  // namespace move_objects_node
}  // namespace xiaohu_robot
#endif