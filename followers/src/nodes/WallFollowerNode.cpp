#include "nodes/WallFollowerNode.hpp"


using namespace std::chrono_literals;


WallFollower::WallFollower() : Node("wall_follower") {
    //declare parameters
    this->declare_parameter("timer_period", rclcpp::ParameterValue(0.05));
    this->declare_parameter("K", rclcpp::ParameterValue(0.0));
    this->declare_parameter("Ti", rclcpp::ParameterValue(0.0));
    this->declare_parameter("Td", rclcpp::ParameterValue(0.0));
    this->declare_parameter("linearSpeed", rclcpp::ParameterValue(0.0));
    this->declare_parameter("maxU", rclcpp::ParameterValue(0.9));
    this->declare_parameter("minirys_namespace", rclcpp::ParameterValue("minirys"));

    this->declare_parameter("left_sensor_offset", rclcpp::ParameterValue(0.0));
    this->declare_parameter("pid_start_side_dist", rclcpp::ParameterValue(0.0));
    this->declare_parameter("front_sensor_turn_end", rclcpp::ParameterValue(0.0));
    this->declare_parameter("turning_side_dist", rclcpp::ParameterValue(0.0));
    this->declare_parameter("not_turning_side_dist", rclcpp::ParameterValue(0.0));
    this->declare_parameter("dist_difference", rclcpp::ParameterValue(0.0));

    this->left_sensor_offset = (float)this->get_parameter("left_sensor_offset").as_double();
    this->pid_start_side_dist = (float)this->get_parameter("pid_start_side_dist").as_double();
    this->front_sensor_turn_end = (float)this->get_parameter("front_sensor_turn_end").as_double();
    this->turning_side_dist = (float)this->get_parameter("turning_side_dist").as_double();
    this->not_turning_side_dist = (float)this->get_parameter("not_turning_side_dist").as_double();
    this->dist_difference = (float)this->get_parameter("dist_difference").as_double();
    std::this_thread::sleep_for(100ms);
    
    //load parameters
    double timer_period = this->get_parameter("timer_period").as_double();
    double K = this->get_parameter("K").as_double();
    double Ti = this->get_parameter("Ti").as_double();
    double Td = this->get_parameter("Td").as_double();
    std::string minirys_namespace = this->get_parameter("minirys_namespace").as_string();
    this->linearSpeed = this->get_parameter("linearSpeed").as_double();
    this->pid = std::make_unique<pid_regulator::PIDRegulator>((float) timer_period, (float) K, (float) Ti, (float) Td);
    this->maxU = (float)this->get_parameter("maxU").as_double();

    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: Ti " << Ti);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: Td " << Td);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: K " << K);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: linear speed " << this->linearSpeed);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: timer_period " << timer_period);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: maxU " << this->maxU);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: minirys_namespace " << minirys_namespace);
    //publishers
    publisher_vocity_ =
    this->create_publisher<geometry_msgs::msg::Twist>("/"+minirys_namespace+"/cmd_vel", 10);
    timer_ = this->create_wall_timer(std::chrono::duration<double>(timer_period), std::bind(&WallFollower::timer_callback, this));
    //subscribers
    subscription_dist5_ = this->create_subscription<sensor_msgs::msg::Range>(
    "/"+minirys_namespace+"/internal/distance_5", 10, std::bind(&WallFollower::left_sensor_callback, this, std::placeholders::_1));
    subscription_dist2_ = this->create_subscription<sensor_msgs::msg::Range>(
    "/"+minirys_namespace+"/internal/distance_2", 10, std::bind(&WallFollower::right_sensor_callback, this, std::placeholders::_1));
    subscription_dist3_ = this->create_subscription<sensor_msgs::msg::Range>(
    "/"+minirys_namespace+"/internal/distance_3", 10, std::bind(&WallFollower::front_sensor_callback, this, std::placeholders::_1));
    subscription_bool_wallf_ = this->create_subscription<std_msgs::msg::Bool>(
    "/"+minirys_namespace+"/is_wall_follower", 10, std::bind(&WallFollower::is_callback, this, std::placeholders::_1));

    program_start_ = std::chrono::high_resolution_clock::now();
    
    }

    
int WallFollower::getTimeToNow(std::chrono::time_point<std::chrono::high_resolution_clock> start_measure_time){
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start_measure_time).count();
}

void WallFollower::timer_callback() {
    if(this->is_working_ && getTimeToNow(program_start_) > INIT_TIME ){
        auto msg_twist = std::make_shared<geometry_msgs::msg::Twist>();
        float u = NO_TURN;
        msg_twist->linear.x = this->linearSpeed;
        if (this->flag_ == PID_WALL_FOLLOWING){
            u = this->pid->pid_aw(this->right_sensor-this->left_sensor,NO_TURN,20.0f, this->maxU);
            if (u >this->maxU){
                u = this->maxU;
            }
            else if(u < -this->maxU){
                u = -this->maxU;
            }
        }
        else if(this->flag_ == TURNING_RIGHT){
            u = -TURN;
            msg_twist->linear.x = this->linearSpeed/2;
        }
        else if(this->flag_ == TURNING_LEFT){
            u = TURN;
            msg_twist->linear.x = this->linearSpeed/2;
        }
        else if(this->flag_ == POSITIONING_AFTER_RIGHT_TURN){
            u = -SLIGHT_TURN;
            
        }
        else if(this->flag_ == POSITIONING_AFTER_LEFT_TURN){
            u = SLIGHT_TURN;
            
        }

        if(((this->right_sensor > turning_side_dist && this->left_sensor < not_turning_side_dist && this->front_sensor < turning_side_dist)
         || (this->right_sensor > this->left_sensor && this->right_sensor-this->left_sensor > dist_difference 
         && this->left_sensor > not_turning_side_dist))){
            this->flag_ = TURNING_RIGHT;
        }

        if(((this->right_sensor < not_turning_side_dist && this->left_sensor > turning_side_dist && this->front_sensor < turning_side_dist)
         || (this->right_sensor < this->left_sensor && this->right_sensor-this->left_sensor < -dist_difference
          && this->right_sensor > not_turning_side_dist))){
            this->flag_ = TURNING_LEFT;
        }

        if(this->front_sensor > front_sensor_turn_end && this->flag_ == TURNING_LEFT){
            this->flag_ = POSITIONING_AFTER_LEFT_TURN;
        }

        if(this->front_sensor > front_sensor_turn_end && this->flag_ == TURNING_RIGHT){
            this->flag_ = POSITIONING_AFTER_RIGHT_TURN;
        }

        if((this->flag_ == POSITIONING_AFTER_RIGHT_TURN || this->flag_ == POSITIONING_AFTER_LEFT_TURN) 
         &&  this->right_sensor <= pid_start_side_dist && this->left_sensor <= pid_start_side_dist){
            this->flag_ = PID_WALL_FOLLOWING;
            this->pid->clear();
        }

        msg_twist->angular.z = u;
        publisher_vocity_->publish(*msg_twist);
    }
}

WallFollower::~WallFollower(){
	auto msg_twist = std::make_shared<geometry_msgs::msg::Twist>();
	publisher_vocity_->publish(*msg_twist);

}

void WallFollower::left_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg) 
{
    this->left_sensor = (float) msg->range + left_sensor_offset;

}

void WallFollower::right_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg) 
{
    this->right_sensor = (float) msg->range;
}

void WallFollower::front_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg) 
{
    this->front_sensor = (float) msg->range;
}

void WallFollower::is_callback(const std_msgs::msg::Bool::SharedPtr msg){
    this->is_working_ = msg->data;
}
