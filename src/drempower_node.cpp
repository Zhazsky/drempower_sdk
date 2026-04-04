#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "drempower_sdk/msg/motor_command.hpp"
#include "drempower_sdk/drempower_driver.hpp"

using namespace std::chrono_literals;

namespace drempower {

class DrempowerNode : public rclcpp::Node {
public:
    DrempowerNode() : Node("drempower_node") {
        // Declare parameters
        this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        this->declare_parameter<int>("baudrate", 115200);
        this->declare_parameter<std::vector<int64_t>>("motor_ids", {1});
        this->declare_parameter<double>("update_rate", 30.0);

        // Initialization parameters
        this->declare_parameter<bool>("init_motor_settings", false);
        this->declare_parameter<bool>("set_zero_position_temp", false);
        this->declare_parameter<double>("min_angle", -180.0);
        this->declare_parameter<double>("max_angle", 180.0);
        this->declare_parameter<bool>("set_angle_range", false);
        this->declare_parameter<bool>("set_angle_range_config", false);
        this->declare_parameter<double>("speed_limit", -1.0);
        this->declare_parameter<double>("torque_limit", -1.0);
        this->declare_parameter<double>("speed_adaptive_limit", -1.0);
        this->declare_parameter<double>("torque_adaptive_limit", -1.0);
        this->declare_parameter<double>("p_gain", -1.0);
        this->declare_parameter<double>("i_gain", -1.0);
        this->declare_parameter<double>("d_gain", -1.0);
        this->declare_parameter<int>("requested_state", -1);
        this->declare_parameter<int>("can_baud_rate", -1);

        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();
        motor_ids_ = this->get_parameter("motor_ids").as_integer_array();
        double update_rate = this->get_parameter("update_rate").as_double();

        // Initialize driver
        if (!driver_.open(port, baudrate)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open port %s", port.c_str());
            rclcpp::shutdown();
            return;
        }

        // Cleanup before starting
        driver_.disableStateFeedback(0);
        driver_.clearUart();

        // Apply initialization settings if requested
        if (this->get_parameter("init_motor_settings").as_bool()) {
            RCLCPP_INFO(this->get_logger(), "Applying initialization motor settings...");
            for (auto id_long : motor_ids_) {
                uint8_t id = static_cast<uint8_t>(id_long);
                
                if (this->get_parameter("set_zero_position_temp").as_bool()) {
                    driver_.setZeroPositionTemp(id);
                }
                
                double min_a = this->get_parameter("min_angle").as_double();
                double max_a = this->get_parameter("max_angle").as_double();
                if (this->get_parameter("set_angle_range").as_bool()) {
                    driver_.setAngleRange(id, static_cast<float>(min_a), static_cast<float>(max_a));
                }
                if (this->get_parameter("set_angle_range_config").as_bool()) {
                    driver_.setAngleRangeConfig(id, static_cast<float>(min_a), static_cast<float>(max_a));
                }
                
                double s_limit = this->get_parameter("speed_limit").as_double();
                if (s_limit > 0) driver_.setSpeedLimit(id, static_cast<float>(s_limit));
                
                double t_limit = this->get_parameter("torque_limit").as_double();
                if (t_limit > 0) driver_.setTorqueLimit(id, static_cast<float>(t_limit));
                
                double sa_limit = this->get_parameter("speed_adaptive_limit").as_double();
                if (sa_limit > 0) driver_.setSpeedAdaptiveLimit(id, static_cast<float>(sa_limit));
                
                double ta_limit = this->get_parameter("torque_adaptive_limit").as_double();
                if (ta_limit > 0) driver_.setTorqueAdaptiveLimit(id, static_cast<float>(ta_limit));
                
                double p = this->get_parameter("p_gain").as_double();
                double i = this->get_parameter("i_gain").as_double();
                double d = this->get_parameter("d_gain").as_double();
                if (p > 0 && i > 0 && d > 0) {
                    driver_.setPID(id, static_cast<float>(p), static_cast<float>(i), static_cast<float>(d));
                }
                
                int state = this->get_parameter("requested_state").as_int();
                if (state >= 1 && state <= 2) driver_.setMode(id, state);
                
                int baud = this->get_parameter("can_baud_rate").as_int();
                if (baud > 0) driver_.setCanBaudRate(id, baud);
            }
        }

        // Publishers
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("motor_states", 10);

        // Subscriptions
        cmd_sub_ = this->create_subscription<drempower_sdk::msg::MotorCommand>(
            "motor_commands", 10, std::bind(&DrempowerNode::commandCallback, this, std::placeholders::_1));

        // Timer for status updates
        auto period = std::chrono::duration<double>(1.0 / update_rate);
        timer_ = this->create_wall_timer(period, std::bind(&DrempowerNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Drempower node started on port %s", port.c_str());
    }

private:
    void commandCallback(const drempower_sdk::msg::MotorCommand::SharedPtr msg) {
        std::vector<uint8_t> ids;
        if (msg->motor_ids.empty()) {
            for (auto id : motor_ids_) ids.push_back(static_cast<uint8_t>(id));
        } else {
            for (auto id : msg->motor_ids) ids.push_back(static_cast<uint8_t>(id));
        }

        if (ids.empty()) {
            RCLCPP_WARN(this->get_logger(), "No motor IDs specified for command");
            return;
        }

        bool multi = (ids.size() > 1);
        bool success = true;

        switch (msg->type) {
            case 0: { // Absolute Angle Position Control
                float angle = msg->values.size() > 0 ? msg->values[0] : 0.0f;
                float speed = msg->values.size() > 1 ? msg->values[1] : 20.0f;
                float param = msg->values.size() > 2 ? msg->values[2] : 10.0f;
                if (multi) {
                    for (auto id : ids) driver_.presetAngle(id, angle, speed, param, msg->mode);
                    success = driver_.syncTrigger(0x08 + msg->mode, 0); // Broadcast trigger
                } else {
                    success = driver_.setAngle(ids[0], angle, speed, param, msg->mode);
                }
                break;
            }
            case 1: { // Speed Control
                float speed = msg->values.size() > 0 ? msg->values[0] : 0.0f;
                float param = msg->values.size() > 1 ? msg->values[1] : 10.0f;
                if (multi) {
                    for (auto id : ids) driver_.presetSpeed(id, speed, param, msg->mode);
                    success = driver_.syncTrigger(0x0B + msg->mode, 0); // Broadcast trigger
                } else {
                    success = driver_.setSpeed(ids[0], speed, param, msg->mode);
                }
                break;
            }
            case 2: { // Torque Control
                float torque = msg->values.size() > 0 ? msg->values[0] : 0.0f;
                float param = msg->values.size() > 1 ? msg->values[1] : 10.0f;
                if (multi) {
                    for (auto id : ids) driver_.presetTorque(id, torque, param, msg->mode);
                    success = driver_.syncTrigger(0x0E + msg->mode, 0); // Broadcast trigger
                } else {
                    success = driver_.setTorque(ids[0], torque, param, msg->mode);
                }
                break;
            }
            case 3: { // Adaptive Angle Control
                float angle = msg->values.size() > 0 ? msg->values[0] : 0.0f;
                float speed = msg->values.size() > 1 ? msg->values[1] : 20.0f;
                float torque = msg->values.size() > 2 ? msg->values[2] : 1.0f;
                for (auto id : ids) {
                    if (!driver_.setAngleAdaptive(id, angle, speed, torque)) success = false;
                }
                break;
            }
            case 4: { // Step Angle (Relative) Control
                float angle = msg->values.size() > 0 ? msg->values[0] : 0.0f;
                float speed = msg->values.size() > 1 ? msg->values[1] : 20.0f;
                float param = msg->values.size() > 2 ? msg->values[2] : 10.0f;
                for (auto id : ids) {
                    if (!driver_.stepAngle(id, angle, speed, param, msg->mode)) success = false;
                }
                break;
            }
            case 5: { // Impedance Control
                float angle = msg->values.size() > 0 ? msg->values[0] : 0.0f;
                float speed = msg->values.size() > 1 ? msg->values[1] : 0.0f;
                float tff = msg->values.size() > 2 ? msg->values[2] : 0.0f;
                float kp = msg->values.size() > 3 ? msg->values[3] : 1.0f;
                float kd = msg->values.size() > 4 ? msg->values[4] : 0.1f;
                for (auto id : ids) {
                    if (!driver_.impedanceControl(id, angle, speed, tff, kp, kd, msg->mode)) success = false;
                }
                break;
            }
            case 6: { // Motion Aid
                float angle = msg->values.size() > 0 ? msg->values[0] : 0.0f;
                float speed = msg->values.size() > 1 ? msg->values[1] : 20.0f;
                float angle_err = msg->values.size() > 2 ? msg->values[2] : 5.0f;
                float speed_err = msg->values.size() > 3 ? msg->values[3] : 2.0f;
                float torque = msg->values.size() > 4 ? msg->values[4] : 1.0f;
                for (auto id : ids) {
                    if (!driver_.motionAid(id, angle, speed, angle_err, speed_err, torque)) success = false;
                }
                break;
            }
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown command type: %d", msg->type);
                break;
        }
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send command of type %d", msg->type);
        }
    }

    void timerCallback() {
        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->now();
        
        for (auto id : motor_ids_) {
            MotorState state;
            if (driver_.getMotorState(static_cast<uint8_t>(id), state)) {
                joint_state_msg.name.push_back("motor_" + std::to_string(id));
                joint_state_msg.position.push_back(state.angle);
                joint_state_msg.velocity.push_back(state.speed);
                joint_state_msg.effort.push_back(state.torque);
            }
        }

        if (!joint_state_msg.name.empty()) {
            joint_pub_->publish(joint_state_msg);
        }
    }

    DrempowerDriver driver_;
    std::vector<int64_t> motor_ids_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Subscription<drempower_sdk::msg::MotorCommand>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace drempower

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<drempower::DrempowerNode>());
    rclcpp::shutdown();
    return 0;
}
