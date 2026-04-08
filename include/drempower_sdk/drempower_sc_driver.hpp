#ifndef DREMPOWER_SDK_DREMPOWER_SC_DRIVER_HPP
#define DREMPOWER_SDK_DREMPOWER_SC_DRIVER_HPP

#include <string>
#include <vector>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace drempower {

struct MotorStateSC {
    uint8_t id;
    float angle;    // °
    float speed;    // r/min
    float torque;   // Nm
};

class DrempowerScDriver {
public:
    DrempowerScDriver();
    ~DrempowerScDriver();

    bool open(const std::string& ifname);
    void close();
    bool isOpen() const { return can_socket_ >= 0; }

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
    bool syncTrigger(uint32_t order_num, uint8_t id = 0); // Broadcast trigger for ID 0
    
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
    bool getMotorState(uint8_t id, MotorStateSC& state);
    float readProperty(uint8_t id, uint16_t address);
    bool writeProperty(uint8_t id, uint16_t address, float value);
    void clearBuffer();

private:
    bool sendCommand(uint8_t id, uint8_t cmd, const std::vector<uint8_t>& data, uint8_t rtr = 0);
    bool receiveData(std::vector<uint8_t>& response, uint8_t expected_id = 0, uint8_t expected_cmd = 0);
    
    std::vector<uint8_t> packData(const std::vector<float>& values, const std::string& types);
    std::vector<float> unpackData(const std::vector<uint8_t>& data, const std::string& types);

    int can_socket_;
    std::mutex can_mutex_;
};

} // namespace drempower

#endif // DREMPOWER_SDK_DREMPOWER_SC_DRIVER_HPP
