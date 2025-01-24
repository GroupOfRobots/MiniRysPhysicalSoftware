#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include <chrono>
#include "pid_regulator/PIDRegulator.hpp"
#include <memory>



using namespace std::chrono_literals;

enum WallFollowerStates{
    PID_WALL_FOLLOWING,
    TURNING_LEFT,
    TURNING_RIGHT,
    POSITIONING_AFTER_LEFT_TURN,
    POSITIONING_AFTER_RIGHT_TURN    
};

class WallFollower: public rclcpp::Node{
    public:
    WallFollower();

    ~WallFollower();
    private:

    int getTimeToNow(std::chrono::time_point<std::chrono::high_resolution_clock> start_measure_time);
    void timer_callback();

    void left_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg); 
    void right_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg); 
    void front_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg); 
    void is_callback(const std_msgs::msg::Bool::SharedPtr msg);
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vocity_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription_dist5_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription_dist2_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription_dist3_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_bool_wallf_;

    float left_sensor = 0.0f;
    float right_sensor = 0.0f;
    float front_sensor = 0.0f;

    std::unique_ptr<pid_regulator::PIDRegulator> pid;
    double linearSpeed;
    float maxU;
    WallFollowerStates flag_ = PID_WALL_FOLLOWING;
    bool is_working_ = true;

    float left_sensor_offset = 0.0f;
    float pid_start_side_dist = 0.0f;
    float front_sensor_turn_end = 0.0f;
    float turning_side_dist = 0.0f;
    float not_turning_side_dist = 0.0f;
    float dist_difference = 0.0f;

    const float TURN = 1.2;
    const float SLIGHT_TURN = 0.1; 
    const float NO_TURN = 0.0;
    const int INIT_TIME = 2000;
    std::chrono::time_point<std::chrono::high_resolution_clock> program_start_;
};
