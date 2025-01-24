#include "minirys_ros2/nodes/MotorsControllerNode.hpp"
#include <iostream>
#include <chrono>
#include <functional>


using namespace std::chrono_literals;
using std::placeholders::_1;

MotorsControllerNode::MotorsControllerNode(rclcpp::NodeOptions options):
	Node("motors_controller_cs", options),
	enabledL(true),
    enabledR(true),
	balancing(false),
	targetBalancing(false),
	targetBalancingPrev(false),
	targetForwardSpeed(0.0),
	targetRotationSpeed(0.0),
	robotAngularPosition(0.0),
	robotAngularVelocity(0.0),
	motorSpeedL(0.0),
	motorSpeedR(0.0),
	standingUpDir(0),
	standingUpPhase(0),
	spinCount(0),
	timeMeasure(100000) {
	// Declare parameters
	// General settings
	this->declare_parameter("updateFrequency", rclcpp::ParameterValue(100.0));
	this->declare_parameter("invertLeftMotor", rclcpp::ParameterValue(false));
	this->declare_parameter("invertRightMotor", rclcpp::ParameterValue(false));
	this->declare_parameter("enableSpeedRegulator", rclcpp::ParameterValue(true));
    this->declare_parameter("wheelRadius", rclcpp::ParameterValue(0.0));
    this->declare_parameter("wheelSeparation", rclcpp::ParameterValue(0.0));

	// Limits
	/// TODO: Adjust the default values of these limits
	this->declare_parameter("maxLinearSpeed", rclcpp::ParameterValue(11.0));
	this->declare_parameter("maxRotationSpeed", rclcpp::ParameterValue(11.0));
	this->declare_parameter("maxWheelSpeed", rclcpp::ParameterValue(11.0));
	this->declare_parameter("maxBalancingAngle", rclcpp::ParameterValue(0.25));
    this->declare_parameter("maxStandUpSpeed", rclcpp::ParameterValue(7.0));

	// Regulator settings
	this->declare_parameter("pidSpeedKp", rclcpp::ParameterValue(0.0));
	this->declare_parameter("pidSpeedTi", rclcpp::ParameterValue(0.0));
	this->declare_parameter("pidSpeedTd", rclcpp::ParameterValue(0.0));
	this->declare_parameter("pidAngleKp", rclcpp::ParameterValue(0.0));
	this->declare_parameter("pidAngleTi", rclcpp::ParameterValue(0.0));
	this->declare_parameter("pidAngleTd", rclcpp::ParameterValue(0.0));

	parametersCallbackHandle = this->add_on_set_parameters_callback(
		std::bind(&MotorsControllerNode::setParametersAtomically, this, std::placeholders::_1));

	// Get and save/use the parameters
    std::this_thread::sleep_for(100ms);
	auto period = std::chrono::duration<double>(1.0 / this->get_parameter("updateFrequency").as_double());
	this->invertLeftMotor = this->get_parameter("invertLeftMotor").as_bool();
	this->invertRightMotor = this->get_parameter("invertRightMotor").as_bool();
	this->enableSpeedRegulator = this->get_parameter("enableSpeedRegulator").as_bool();
    this->wheelRadius = this->get_parameter("wheelRadius").as_double();
    this->wheelSeparation = this->get_parameter("wheelSeparation").as_double();

	this->maxLinearSpeed = this->get_parameter("maxLinearSpeed").as_double();
	this->maxRotationSpeed = this->get_parameter("maxRotationSpeed").as_double();
	this->maxWheelSpeed = this->get_parameter("maxWheelSpeed").as_double();
	this->maxBalancingAngle = this->get_parameter("maxBalancingAngle").as_double();
    this->maxStandUpSpeed = this->get_parameter("maxStandUpSpeed").as_double();

	auto pidSpeedKp = this->get_parameter("pidSpeedKp").as_double();
	auto pidSpeedTi = this->get_parameter("pidSpeedTi").as_double();
	auto pidSpeedTd = this->get_parameter("pidSpeedTd").as_double();
	auto pidAngleKp = this->get_parameter("pidAngleKp").as_double();
	auto pidAngleTi = this->get_parameter("pidAngleTi").as_double();
	auto pidAngleTd = this->get_parameter("pidAngleTd").as_double();

    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: pidSpeedKp " << pidSpeedKp);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: pidSpeedTi " << pidSpeedTi);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: pidSpeedTd " << pidSpeedTd);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: pidAngleKp " << pidAngleKp);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: pidAngleTi " << pidAngleTi);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: pidAngleTd " << pidAngleTd);
	
	this->speedPidRegulator = std::make_unique<pid_regulator::PIDRegulator>((float) period.count(), (float) pidSpeedKp, (float) pidSpeedTi, (float) pidSpeedTd);
	this->anglePidRegulator = std::make_unique<pid_regulator::PIDRegulator>((float) period.count(), (float) pidAngleKp, (float) pidAngleTi, (float) pidAngleTd);

	// Setup the clock (for standing up)
	this->steadyROSClock = rclcpp::Clock(RCL_STEADY_TIME);

	// Setup the subscriptions, publishers and timers
	this->velocityCommandSubscriber = this->create_subscription<geometry_msgs::msg::Twist>(
		"cmd_vel",
		10,
		std::bind(&MotorsControllerNode::receiveVelocityCommand, this, _1)
	);
	this->balanceModeSubscriber = this->create_subscription<std_msgs::msg::Bool>(
		"balance_mode",
		10,
		std::bind(&MotorsControllerNode::receiveBalanceMode, this, _1)
	);
	this->angularPoseSubscriber = this->create_subscription<minirys_msgs::msg::AngularPose>(
		"internal/angular_pose",
		10,
		std::bind(&MotorsControllerNode::receiveAngularPose, this, _1)
	);
	this->motorSpeedLSubscriber = this->create_subscription<std_msgs::msg::Float64>(
		"internal/motor_speed_l",
		10,
		std::bind(&MotorsControllerNode::receiveMotorLSpeed, this, _1)
	);
	this->motorSpeedRSubscriber = this->create_subscription<std_msgs::msg::Float64>(
		"internal/motor_speed_r",
		10,
		std::bind(&MotorsControllerNode::receiveMotorRSpeed, this, _1)
	);
	this->motorStatusLSubscriber = this->create_subscription<minirys_msgs::msg::MotorDriverStatus>(
		"internal/motor_status_l",
		10,
		std::bind(&MotorsControllerNode::receiveMotorLStatus, this, _1)
	);
	this->motorStatusRSubscriber = this->create_subscription<minirys_msgs::msg::MotorDriverStatus>(
		"internal/motor_status_r",
		10,
		std::bind(&MotorsControllerNode::receiveMotorRStatus, this, _1)
	);

	this->motorCommandPublisher = this->create_publisher<minirys_msgs::msg::MotorCommand>("internal/motor_command", 10);

	this->updateTimer = this->create_wall_timer(period, std::bind(&MotorsControllerNode::update, this));
}

MotorsControllerNode::~MotorsControllerNode() {
	this->timeMeasure.saveHistCSV("/tmp/minirys_motorscontrollernode_hist.csv");
	this->timeMeasure.saveTopNCSV("/tmp/minirys_motorscontrollernode_top.csv");
}

void MotorsControllerNode::update() {
	this->spinCount++;
	int64_t latency = 0;

	if (rcl_timer_get_time_since_last_call(&(*this->updateTimer->get_timer_handle()), &latency) == RCL_RET_OK) {
		this->timeMeasure.add(latency);
	} else {
		RCLCPP_ERROR_STREAM(this->get_logger(), "Error getting time since last call!");
	}

	if (!this->enabledL || !this->enabledR) {
		auto message = minirys_msgs::msg::MotorCommand();
		message.speed_l = 0.0f;
		message.speed_r = 0.0f;
		message.enable = false;
		this->motorCommandPublisher->publish(message);
		return;
	}

	if (this->targetBalancing && !this->targetBalancingPrev) {
		this->standingUpDir = 0;
		this->anglePidRegulator->clear();
       	this->speedPidRegulator->clear();
	}
	this->targetBalancingPrev = this->targetBalancing;

	std::pair<double, double> speeds;

	if (this->balancing && std::abs(this->robotAngularPosition) > 0.7) {
		this->balancing = false;
	}
	if (this->balancing && !this->targetBalancing) {
		this->balancing = false;
	}
	if (!this->balancing) {
		this->anglePidRegulator->clear();
       	this->speedPidRegulator->clear();
	}

	if (this->balancing) {
		speeds = this->calculateSpeedsBalancing();
	} else if (this->targetBalancing) {
		speeds = this->standUp();
        //this->angleRegulator.zero();
	} else {
		speeds = this->calculateSpeedsFlat();
	}

	auto message = minirys_msgs::msg::MotorCommand();
	message.header.stamp = this->get_clock()->now();
	message.header.frame_id = "motors_controller";
	message.speed_l = speeds.first * (this->invertLeftMotor ? 1 : -1);
	message.speed_r = speeds.second * (this->invertRightMotor ? 1 : -1);
	//message.enable = true;
	this->motorCommandPublisher->publish(message);
}

void MotorsControllerNode::receiveVelocityCommand(const geometry_msgs::msg::Twist::SharedPtr message) {
	this->targetForwardSpeed = std::min(std::max(message->linear.x/this->wheelRadius, -this->maxLinearSpeed), this->maxLinearSpeed);
	this->targetRotationSpeed = std::min(std::max(message->angular.z*this->wheelSeparation/(2*this->wheelRadius), -this->maxRotationSpeed), this->maxRotationSpeed);
//    RCLCPP_INFO_STREAM(this->get_logger(), "Forward Speed: " << this->targetForwardSpeed);
//    RCLCPP_INFO_STREAM(this->get_logger(), "Rotation Speed: " << this->targetRotationSpeed);

}

void MotorsControllerNode::receiveBalanceMode(const std_msgs::msg::Bool::SharedPtr message) {
	this->targetBalancing = message->data;
}

void MotorsControllerNode::receiveAngularPose(const minirys_msgs::msg::AngularPose::SharedPtr message) {
	this->robotAngularPosition = message->angular_position;
	this->robotAngularVelocity = message->angular_velocity;
}

void MotorsControllerNode::receiveMotorLSpeed(const std_msgs::msg::Float64::SharedPtr message) {
	this->motorSpeedL = message->data * (this->invertLeftMotor ? -1 : 1);
}

void MotorsControllerNode::receiveMotorRSpeed(const std_msgs::msg::Float64::SharedPtr message) {
	this->motorSpeedR = message->data * (this->invertRightMotor ? -1 : 1);
}

void MotorsControllerNode::receiveMotorLStatus(const minirys_msgs::msg::MotorDriverStatus::SharedPtr message) {
	if (!message->undervoltage || !message->thermal_warning || !message->thermal_shutdown || !message->overcurrent) {
        if (!message->undervoltage) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Detected undervoltage on left driver");
        }
        if (!message->thermal_warning) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Detected thermal warning on left driver");
        }
        if (!message->thermal_shutdown) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Detected thermal shutdown on left driver");
        }
        if (!message->overcurrent) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Detected overcurrent on left driver");
        }
        RCLCPP_ERROR_STREAM(this->get_logger(), "Motors stopped");
        this->enabledL = false;
	}
}

void MotorsControllerNode::receiveMotorRStatus(const minirys_msgs::msg::MotorDriverStatus::SharedPtr message) {
	if (!message->undervoltage || !message->thermal_warning || !message->thermal_shutdown || !message->overcurrent) {
        if (!message->undervoltage) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Detected undervoltage on right driver");
        }
        if (!message->thermal_warning) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Detected thermal warning on right driver");
        }
        if (!message->thermal_shutdown) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Detected thermal shutdown on right driver");
        }
        if (!message->overcurrent) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Detected overcurrent on right driver");
        }
        RCLCPP_ERROR_STREAM(this->get_logger(), "Motors stopped");
		this->enabledR = false;
	}
}

std::pair<double, double> MotorsControllerNode::calculateSpeedsFlat() const {
	double outputL = this->targetForwardSpeed + this->targetRotationSpeed;
	double outputR = this->targetForwardSpeed - this->targetRotationSpeed;
    //double currentSpeed = (this->motorSpeedL + this->motorSpeedR) / 2;
    //RCLCPP_INFO_STREAM(this->get_logger(), "CurrentSpeed: " << currentSpeed);
    //RCLCPP_INFO_STREAM(this->get_logger(), "l Speed: " << this->motorSpeedL);
	return {
		std::min(std::max(outputL, -this->maxWheelSpeed), this->maxWheelSpeed),
		std::min(std::max(outputR, -this->maxWheelSpeed), this->maxWheelSpeed)
	};
}

std::pair<double, double> MotorsControllerNode::calculateSpeedsBalancing() {
	double outputTargetAngle = 0.0f;
    double currentSpeed = (this->motorSpeedL + this->motorSpeedR) / 2;
	if (this->enableSpeedRegulator) {
	outputTargetAngle = this->speedPidRegulator->pid_aw(currentSpeed, this->targetForwardSpeed, 10.0f, 0.25);
	outputTargetAngle = std::min(std::max(-outputTargetAngle, -0.25), 0.25);
	}
    	double outputWheelSpeed = this->anglePidRegulator->pid_aw(this->robotAngularPosition,outputTargetAngle, 10.0f, this->maxWheelSpeed);
	return {

		std::min(std::max(outputWheelSpeed + this->targetRotationSpeed, -this->maxWheelSpeed), this->maxWheelSpeed),
                std::min(std::max(outputWheelSpeed - this->targetRotationSpeed, -this->maxWheelSpeed), this->maxWheelSpeed)
	};
}

std::pair<double, double> MotorsControllerNode::standUp() {
	if (this->standingUpDir == 0) {
		this->standingUpDir = this->robotAngularPosition < 0 ? -1 : 1;
		this->standingUpPhase = 0;
		this->anglePidRegulator->clear();
       	this->speedPidRegulator->clear();
		this->standingUpStart = this->steadyROSClock.now();
	}

	if (this->standingUpPhase == 0) {
		// Phase 0: get up to speed
		if (this->steadyROSClock.now() - this->standingUpStart >= 500ms) {
			// After 500ms switch to phase 1
			this->standingUpDir = -this->standingUpDir;
			this->standingUpPhase = 1;
		}
	} else {
		// Phase 1: reverse the direction and get up using the momentum
		if (this->robotAngularPosition >= -0.3 && this->robotAngularPosition <= 0.3) {
			// We're up, switch to balancing regulation
			this->standingUpDir = 0;
			this->balancing = true;
		} else if (this->steadyROSClock.now() - this->standingUpStart >= 1500ms) {
			// We didn't have enough speed, try again
			this->standingUpDir = 0;
		}
	}

	return {
		this->standingUpDir * this->maxStandUpSpeed,
		this->standingUpDir * this->maxStandUpSpeed
	};
}

rcl_interfaces::msg::SetParametersResult
MotorsControllerNode::setParametersAtomically(const std::vector<rclcpp::Parameter> &parameters) {
	for (const auto &parameter : parameters)
	{
		const std::string &name = parameter.get_name();
		const rclcpp::ParameterValue parameter1 = parameter.get_parameter_value();
		const double value = parameter1.get<double>();

		RCLCPP_INFO_STREAM(this->get_logger(), "Got param from service: " << name << "  " << value);

		if (name == "pidSpeedKp") {
			this->speedPidRegulator->setK(value);
        } else if (name == "pidSpeedTi") {
			this->speedPidRegulator->setTi(value);
        } else if (name == "pidSpeedTd") {
			this->speedPidRegulator->setTd(value);
		} else if (name == "pidAngleKp") {
			this->anglePidRegulator->setK(value);
		} else if (name == "pidAngleTi") {
			this->anglePidRegulator->setTi(value);
		} else if (name == "pidAngleTd") {
			this->anglePidRegulator->setTd(value);
		} else {
			RCLCPP_INFO_STREAM(this->get_logger(), "Unknown param");
        }

	}

	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;
	result.reason = "success";
	return result;
}