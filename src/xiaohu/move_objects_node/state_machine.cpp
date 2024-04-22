#include "xiaohu/move_objects_node/state_machine.hpp"

using namespace xiaohu_robot::move_objects_node;

StateMachine::StateMachine(std::string base_name):
    node_handle{}, timing{0_s}, task_state{TaskState::READY}, move_tasks{}, base_name{std::move(base_name)},
    velocity_publisher{node_handle.advertise<velocity_msg>("/cmd_vel", 10)},
    behaviors_publisher{node_handle.advertise<string_msg>("/wpb_home/behaviors", 10)},
    waypoint_publisher{node_handle.advertise<string_msg>("/waterplus/navi_waypoint", 10)},
    grab_object_publisher{node_handle.advertise<position_msg>("/wpb_home/grab_action", 1)},
    manipulatior_control_publisher{node_handle.advertise<manipulator_control_msg>("/wpb_home/mani_ctrl", 10)},
    object_coordinate_subscriber{node_handle.subscribe<coordinates_msg_ptr>(
        "/wpb_home/objects_3d", 10, &StateMachine::process_object_detect_message, this
    )},
    navigation_result_subscriber{node_handle.subscribe<string_msg_ptr>(
        "/waterplus/navi_result", 10, &StateMachine::process_navigation_result_message, this
    )},
    grab_result_subscriber{node_handle.subscribe<string_msg_ptr>(
        "/wpb_home/grab_result", 10, &StateMachine::process_grab_result_message, this
    )},
    move_task_subscriber{node_handle.subscribe<move_task_msg_ptr>(
        "/xiaohu/move_tasks", 10, &StateMachine::process_move_task_message, this
    )},
    cancellation_subscriber{
        node_handle.subscribe<string_msg_ptr>("/xiaohu/cancel", 1, &StateMachine::process_cancel_task_message, this)
    } {
    sleep(1);  // Waiting for the topic to be published/subscribed properly.
}

void StateMachine::run() {
    ros::Rate loop_rate{loops_per_second};
    constexpr auto estimate_initial_position_delay{15};
    ROS_INFO("Please estimate initial position in %d seconds.", estimate_initial_position_delay);
    sleep(estimate_initial_position_delay);

    ROS_INFO("Next state: READY");
    while (ros::ok()) {
        switch (get_current_task_state()) {
        case TaskState::READY:
            if (!move_tasks.empty()) {
                go_to_storage();
            }
            break;
        case TaskState::GO_TO_STORAGE_PLACE:
            // Should be processed by process_navigation_result_callback
            break;
        case TaskState::OBJECT_DETECT:
            // Should be processed by process_object_detect_callback
            break;
        case TaskState::GRAB_OBJECT:
            // Should be processed by process_grab_result_callback
            break;
        case TaskState::GO_TO_DROP_PLACE:
            // Should be processed by process_navigation_result_callback
            break;
        case TaskState::PUT_DOWN_OBJECT:
            put_down_object();
            break;
        case TaskState::STEP_BACKWARD:
            step_backward();
            break;
        case TaskState::RETRACT_MANIPULATOR:
            retract_manipulaor();
            break;
        case TaskState::DONE:
            if (!move_tasks.empty())
                go_to_storage();
            else
                go_to_base();
            break;
        case TaskState::GO_TO_BASE:
            // Should be processed by process_navigation_result_callback
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

StateMachine::MoveTask StateMachine::get_current_move_task() const {
    return move_tasks.front();
}

StateMachine::TaskState StateMachine::get_current_task_state() const {
    return task_state;
}

void StateMachine::set_next_task_state(TaskState next_state) {
    task_state = next_state;
}

StateMachine::duration StateMachine::get_timing() const {
    return timing;
}

void StateMachine::increment_timing() {
    // auto old_timing{timing};
    timing += duration{1.0 / loops_per_second, UnitDuration::seconds};
    // ROS_INFO("Waiting: %s -> %s", old_timing.to_string().c_str(), timing.to_string().c_str());
}

void StateMachine::reset_timing() {
    timing.set_value(0);
};

void StateMachine::go_to_base() {
    set_next_task_state(TaskState::GO_TO_BASE);
    ROS_INFO("Next state: GO TO BASE");
    notify_navigate_robot(base_name);
}

void StateMachine::go_to_storage() {
    set_next_task_state(TaskState::GO_TO_STORAGE_PLACE);
    ROS_INFO("Next state: GO TO STORAGE PLACE %s", get_current_move_task().storage_place_name.c_str());
    notify_navigate_robot(get_current_move_task().storage_place_name.c_str());
}

void StateMachine::put_down_object() {
    increment_timing();
    if (get_timing() > 5_s) {
        ROS_INFO("Have put down the object.");
        reset_timing();
        set_next_task_state(TaskState::STEP_BACKWARD);
        ROS_INFO("Next state: STEP BACKWARD.");
    }
}

void StateMachine::step_backward() {
    notify_control_robot_speed(-0.1);
    increment_timing();
    if (get_timing() > 5_s) {
        ROS_INFO("Have stepped backward.");
        reset_timing();
        notify_control_robot_speed(0);
        ROS_INFO("Stop moving.");
        notify_control_robot_manipulator(ArmControl{0_cm});
        set_next_task_state(TaskState::RETRACT_MANIPULATOR);
        ROS_INFO("Retracting the manipulator.");
    }
}

void StateMachine::retract_manipulaor() {
    increment_timing();
    if (get_timing() > 5_s) {
        ROS_INFO("Have retracted the manipulator.");
        reset_timing();
        ROS_INFO(
            "Task %s -> %s: DONE!", get_current_move_task().storage_place_name.c_str(),
            get_current_move_task().storage_place_name.c_str()
        );
        set_next_task_state(TaskState::DONE);
        ROS_INFO("Next state: DONE");
        move_tasks.pop_front();
        show_tasks();
    }
}

void StateMachine::show_tasks() const {
    ROS_INFO("Task count:%zd", move_tasks.size());
    auto task_index{1};
    for (auto const& task : move_tasks) {
        ROS_INFO("Task %d: %s -> %s", task_index, task.storage_place_name.c_str(), task.drop_place_name.c_str());
    }
}

void StateMachine::process_object_detect_message(coordinates_msg_ptr coordinates_ptr) {
    if (get_current_task_state() != TaskState::OBJECT_DETECT)
        return;

    ROS_INFO("Start object detection.");
    display_detected_objects(coordinates_ptr);

    constexpr auto nearest_object_index{0};
    notify_grab_object(coordinates_ptr, nearest_object_index);

    notify_change_robot_behavior(Behavior::STOP_OBJECT_DETECTION);
    ROS_INFO("Stop object detection.");
    set_next_task_state(TaskState::GRAB_OBJECT);
    ROS_INFO("Next state: GRAB OBJECT");
}

void StateMachine::process_navigation_result_message(string_msg_ptr message_ptr) {
    if (message_ptr->data != "done")
        return;
    if (get_current_task_state() == TaskState::GO_TO_STORAGE_PLACE) {
        ROS_INFO(
            "[navigation_result_callback] Arrived storage place: %s!",
            get_current_move_task().storage_place_name.c_str()
        );
        notify_change_robot_behavior(Behavior::START_OBJECT_DETECTION);
        set_next_task_state(TaskState::OBJECT_DETECT);
        ROS_INFO("Next state: OBJECT DETECT");
    }
    else if (get_current_task_state() == TaskState::GO_TO_DROP_PLACE) {
        ROS_INFO(
            "[navigation_result_callback] Arrived drop place: %s!", get_current_move_task().drop_place_name.c_str()
        );
        notify_control_robot_manipulator(GripperControl{15_cm});
        set_next_task_state(TaskState::PUT_DOWN_OBJECT);
        ROS_INFO("Next state: PUT DOWN OBJECT");
        reset_timing();
    }
    else if (get_current_task_state() == TaskState::GO_TO_BASE) {
        ROS_INFO("[navigation_result_callback] Arrived base: %s!", base_name.c_str());
        set_next_task_state(TaskState::READY);
        ROS_INFO("Next state: READY");
    }
}

void StateMachine::process_grab_result_message(string_msg_ptr message_ptr) {
    if (get_current_task_state() != TaskState::GRAB_OBJECT)
        return;
    if (message_ptr->data != "done")
        return;
    ROS_INFO("[grab_result_callback] grab object done!");
    notify_navigate_robot(get_current_move_task().drop_place_name);
    set_next_task_state(TaskState::GO_TO_DROP_PLACE);
    ROS_INFO("Next state: GO TO DROP PLACE %s", get_current_move_task().drop_place_name.c_str());
}

void StateMachine::process_move_task_message(move_task_msg_ptr message_ptr) {
    move_tasks.emplace_back(MoveTask{message_ptr->storage_place_name, message_ptr->drop_place_name});
    ROS_INFO(
        "Added task: move an object from %s to %s", message_ptr->storage_place_name.c_str(),
        message_ptr->drop_place_name.c_str()
    );
}

void StateMachine::process_cancel_task_message(string_msg_ptr message_ptr) {
    move_tasks.clear();
    reset_timing();
    ROS_INFO("Cancelled all tasks");
    set_next_task_state(TaskState::DONE);
}

void StateMachine::notify_control_robot_speed(double axis_x_linear_speed) {
    velocity_publisher.publish(create_velocity_command(axis_x_linear_speed));
}

void StateMachine::notify_change_robot_behavior(Behavior behavior) {
    behaviors_publisher.publish(create_behavior_message(behavior));
}

void StateMachine::notify_navigate_robot(std::string waypoint_name) {
    waypoint_publisher.publish(create_waypoint_message(std::move(waypoint_name)));
}

void StateMachine::notify_control_robot_manipulator(ManipulatorControl const& plan) {
    manipulatior_control_publisher.publish(plan.to_message());
}

void StateMachine::notify_grab_object(coordinates_msg_ptr coordinates_ptr, size_t index) {
    grab_object_publisher.publish(
        create_position_message(coordinates_ptr->x[index], coordinates_ptr->y[index], coordinates_ptr->z[index])
    );
}

StateMachine::velocity_msg StateMachine::create_velocity_command(double axis_x_linear_speed) {
    velocity_msg velocity_command{};
    velocity_command.linear.x = axis_x_linear_speed;
    return velocity_command;
}

StateMachine::string_msg StateMachine::create_waypoint_message(std::string waypoint_name) {
    string_msg message{};
    message.data = std::move(waypoint_name);
    return message;
}

StateMachine::string_msg StateMachine::create_behavior_message(Behavior behavior) {
    string_msg message{};
    switch (behavior) {
    case Behavior::START_OBJECT_DETECTION:
        message.data = "object_detect start";
        break;
    case Behavior::STOP_OBJECT_DETECTION:
        message.data = "object_detect stop";
        break;
    }
    return message;
}

StateMachine::position_msg StateMachine::create_position_message(double x, double y, double z) {
    position_msg message{};
    message.position.x = x;
    message.position.y = y;
    message.position.z = z;
    return message;
}

void StateMachine::display_detected_objects(coordinates_msg_ptr coordinates_ptr) {
    size_t const total_amount{coordinates_ptr->name.size()};
    ROS_INFO("[process_object_detect_callback] Total amount of objects: %zd", total_amount);
    for (size_t object_id{0}; object_id < total_amount; ++object_id) {
        ROS_INFO(
            "[process_object_detect_callback] object %s's coordinate: (%.2f, %.2f, %.2f)%s",
            coordinates_ptr->name[object_id].c_str(), coordinates_ptr->x[object_id], coordinates_ptr->y[object_id],
            coordinates_ptr->z[object_id], object_id == 0 ? " which is nearest and will be picked" : ""
        );
    }
}