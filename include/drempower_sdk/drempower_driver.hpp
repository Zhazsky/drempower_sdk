#ifndef DREMPOWER_SDK_DREMPOWER_DRIVER_HPP
#define DREMPOWER_SDK_DREMPOWER_DRIVER_HPP

#include <string>
#include <vector>
#include <cstdint>
#include <boost/asio.hpp>
#include <iostream>
#include <mutex>

namespace drempower {

struct MotorState {
    uint8_t id;
    float angle;    // °
    float speed;    // r/min
    float torque;   // Nm
};

class DrempowerDriver {
public:
    DrempowerDriver();
    ~DrempowerDriver();

    bool open(const std::string& port, uint32_t baudrate);
    void close();
    bool isOpen() const { return serial_port_ && serial_port_->is_open(); }

    // Control Modes
    bool setAngle(uint8_t id, float angle, float speed, float param, uint8_t mode);
    bool setSpeed(uint8_t id, float speed, float param, uint8_t mode);
    bool setTorque(uint8_t id, float torque, float param, uint8_t mode);

    // Preset Modes (for synchronized multi-motor control)
    bool presetAngle(uint8_t id, float angle, float speed, float param, uint8_t mode);
    bool presetSpeed(uint8_t id, float speed, float param, uint8_t mode);
    bool presetTorque(uint8_t id, float torque, float param, uint8_t mode);
    
    // New Advanced Control Modes
    bool setAngleAdaptive(uint8_t id, float angle, float speed, float torque);
    bool stepAngle(uint8_t id, float angle, float speed, float param, uint8_t mode);
    bool impedanceControl(uint8_t id, float angle, float speed, float tff, float kp, float kd, uint8_t mode);
    bool motionAid(uint8_t id, float angle, float speed, float angle_err, float speed_err, float torque);

    // Synchronized Multi-Motor Trigger
    bool syncTrigger(uint32_t order_num, uint8_t id = 0); // Broadcase trigger for ID 0
    
    bool setZeroPosition(uint8_t id);
    bool setZeroPositionTemp(uint8_t id);
    bool estop(uint8_t id = 0);
    
    // Configuration
    bool setAngleRange(uint8_t id, float min_angle, float max_angle);
    bool disableAngleRange(uint8_t id);
    bool setAngleRangeConfig(uint8_t id, float min_angle, float max_angle);
    bool disableAngleRangeConfig(uint8_t id);
    bool setSpeedLimit(uint8_t id, float speed_limit);
    bool setTorqueLimit(uint8_t id, float torque_limit);
    bool setSpeedAdaptiveLimit(uint8_t id, float speed_adaptive);
    bool setTorqueAdaptiveLimit(uint8_t id, float torque_adaptive);
    bool setPID(uint8_t id, float p, float i, float d);
    bool setMode(uint8_t id, uint32_t mode);
    bool setCanBaudRate(uint8_t id, uint32_t baud_rate);

    bool disableStateFeedback(uint8_t id = 0);
    bool enableStateFeedback(uint8_t id = 0);
    bool setStateFeedbackRate(uint8_t id, uint16_t interval_ms);
    bool saveConfig(uint8_t id);

    // Data Reading
    bool getMotorState(uint8_t id, MotorState& state);
    float readProperty(uint8_t id, uint16_t address);
    bool writeProperty(uint8_t id, uint16_t address, float value);
    void clearUart();

private:
    bool sendCommand(uint8_t id, uint8_t cmd, const std::vector<uint8_t>& data, uint8_t rtr = 0);
    bool receiveData(std::vector<uint8_t>& response);
    
    std::vector<uint8_t> packData(const std::vector<float>& values, const std::string& types);
    std::vector<float> unpackData(const std::vector<uint8_t>& data, const std::string& types);

    boost::asio::io_context io_context_;
    std::unique_ptr<boost::asio::serial_port> serial_port_;
    std::mutex serial_mutex_;
};

} // namespace drempower

#endif // DREMPOWER_SDK_DREMPOWER_DRIVER_HPP
