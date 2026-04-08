#include "drempower_sdk/drempower_sc_driver.hpp"
#include "drempower_sdk/parameter_interface.hpp"
#include <cstring>
#include <chrono>
#include <thread>
#include <limits>
#include <cmath>
#include <sstream>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <poll.h>

namespace drempower {

DrempowerScDriver::DrempowerScDriver() : can_socket_(-1) {}

DrempowerScDriver::~DrempowerScDriver() {
    close();
}

bool DrempowerScDriver::open(const std::string& ifname) {
    struct sockaddr_can addr;
    struct ifreq ifr;

    if ((can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        std::cerr << "Error while opening CAN socket" << std::endl;
        return false;
    }

    std::strcpy(ifr.ifr_name, ifname.c_str());
    ioctl(can_socket_, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "Error in socket bind" << std::endl;
        ::close(can_socket_);
        can_socket_ = -1;
        return false;
    }

    return true;
}

void DrempowerScDriver::close() {
    if (can_socket_ >= 0) {
        ::close(can_socket_);
        can_socket_ = -1;
    }
}

bool DrempowerScDriver::setAngle(uint8_t id, float angle, float speed, float param, uint8_t mode) {
    if (!presetAngle(id, angle, speed, param, mode)) return false;
    uint32_t order_num = 0x08 + mode;
    auto trigger_data = packData({(float)order_num, 1.0f}, "u32 u16");
    return sendCommand(id, 0x08, trigger_data);
}

bool DrempowerScDriver::presetAngle(uint8_t id, float angle, float speed, float param, uint8_t mode) {
    float factor = 0.01f;
    uint8_t cmd = 0x19;
    std::vector<uint8_t> data;
    
    if (mode == 0) {
        cmd = 0x19;
        int16_t s16_speed = static_cast<int16_t>(std::abs(speed) / factor);
        int16_t s16_width = static_cast<int16_t>(std::abs(param) / factor);
        data = packData({angle, (float)s16_speed, (float)s16_width}, "f s16 s16");
    } else if (mode == 1) {
        cmd = 0x1A;
        int16_t s16_speed = static_cast<int16_t>(std::abs(speed) / factor);
        int16_t s16_accel = static_cast<int16_t>(std::abs(param) / factor);
        data = packData({angle, (float)s16_speed, (float)s16_accel}, "f s16 s16");
    } else if (mode == 2) {
        cmd = 0x1B;
        int16_t s16_speed_ff = static_cast<int16_t>(speed / factor);
        int16_t s16_torque_ff = static_cast<int16_t>(param / factor);
        data = packData({angle, (float)s16_speed_ff, (float)s16_torque_ff}, "f s16 s16");
    }
    return sendCommand(id, cmd, data);
}

bool DrempowerScDriver::setSpeed(uint8_t id, float speed, float param, uint8_t mode) {
    if (!presetSpeed(id, speed, param, mode)) return false;
    uint32_t order_num = 0x0B + mode;
    auto trigger_data = packData({(float)order_num, 1.0f}, "u32 u16");
    return sendCommand(id, 0x08, trigger_data);
}

bool DrempowerScDriver::presetSpeed(uint8_t id, float speed, float param, uint8_t mode) {
    float factor = 0.01f;
    uint8_t cmd = 0x1C;
    std::vector<uint8_t> data;
    if (mode == 0) {
        int16_t s16_torque = static_cast<int16_t>(param / factor);
        uint16_t u16_input_mode = 1;
        data = packData({speed, (float)s16_torque, (float)u16_input_mode}, "f s16 u16");
    } else {
        int16_t s16_ramp_rate = static_cast<int16_t>(param / factor);
        uint16_t u16_input_mode = 2;
        data = packData({speed, (float)s16_ramp_rate, (float)u16_input_mode}, "f s16 u16");
    }
    return sendCommand(id, cmd, data);
}

bool DrempowerScDriver::setTorque(uint8_t id, float torque, float param, uint8_t mode) {
    if (!presetTorque(id, torque, param, mode)) return false;
    uint32_t order_num = 0x0E + mode;
    auto trigger_data = packData({(float)order_num, 1.0f}, "u32 u16");
    return sendCommand(id, 0x08, trigger_data);
}

bool DrempowerScDriver::presetTorque(uint8_t id, float torque, float param, uint8_t mode) {
    float factor = 0.01f;
    uint8_t cmd = 0x1D;
    std::vector<uint8_t> data;
    if (mode == 0) {
        uint16_t u16_input_mode = 1;
        data = packData({torque, 0.0f, (float)u16_input_mode}, "f s16 u16");
    } else {
        int16_t s16_ramp_rate = static_cast<int16_t>(param / factor);
        uint16_t u16_input_mode = 6;
        data = packData({torque, (float)s16_ramp_rate, (float)u16_input_mode}, "f s16 u16");
    }
    return sendCommand(id, cmd, data);
}

bool DrempowerScDriver::setAngleAdaptive(uint8_t id, float angle, float speed, float torque) {
    float factor = 0.01f;
    uint8_t cmd = 0x0B;
    int16_t s16_speed = static_cast<int16_t>(std::abs(speed) / factor);
    int16_t s16_torque = static_cast<int16_t>(std::abs(torque) / factor);
    auto data = packData({angle, (float)s16_speed, (float)s16_torque}, "f s16 s16");
    return sendCommand(id, cmd, data);
}

bool DrempowerScDriver::stepAngle(uint8_t id, float angle, float speed, float param, uint8_t mode) {
    float factor = 0.01f;
    uint8_t cmd = 0x0C;
    std::vector<uint8_t> data;
    uint32_t order_num = 0;

    if (mode == 0) {
        if (param > 300) param = 300;
        int16_t s16_speed = static_cast<int16_t>(std::abs(speed) / factor);
        int16_t s16_width = static_cast<int16_t>(std::abs(param) / factor);
        data = packData({angle, (float)s16_speed, (float)s16_width}, "f s16 s16");
        order_num = 0x10;
    } else if (mode == 1) {
        int16_t s16_speed = static_cast<int16_t>(std::abs(speed) / factor);
        int16_t s16_accel = static_cast<int16_t>(std::abs(param) / factor);
        data = packData({angle, (float)s16_speed, (float)s16_accel}, "f s16 s16");
        order_num = 0x11;
    } else if (mode == 2) {
        int16_t s16_speed_ff = static_cast<int16_t>(speed / factor);
        int16_t s16_torque_ff = static_cast<int16_t>(param / factor);
        data = packData({angle, (float)s16_speed_ff, (float)s16_torque_ff}, "f s16 s16");
        order_num = 0x12;
    }

    if (!sendCommand(id, cmd, data)) return false;
    auto trigger_data = packData({(float)order_num, 1.0f}, "u32 u16");
    return sendCommand(id, 0x08, trigger_data);
}

bool DrempowerScDriver::impedanceControl(uint8_t id, float angle, float speed, float tff, float kp, float kd, uint8_t mode) {
    float factor = 0.001f;
    kp = std::abs(kp);
    kd = std::abs(kd);
    if (kp > 20.0f) kp = 20.0f;
    if (kd > 20.0f) kd = 20.0f;

    float angle_set = angle;
    if (mode == 1) {
        if (kp != 0.0f) {
            angle_set = (-kd * speed - tff) / kp + angle;
        } else {
            return false;
        }
    }

    float f_factor = 0.01f;
    int16_t s16_speed_ff = static_cast<int16_t>(speed / f_factor);
    int16_t s16_torque_ff = static_cast<int16_t>(tff / f_factor);
    auto preset_data = packData({angle_set, (float)s16_speed_ff, (float)s16_torque_ff}, "f s16 s16");
    if (!sendCommand(id, 0x1B, preset_data)) return false;

    uint32_t order_num = 0x15;
    auto trigger_data = packData({(float)order_num, (float)static_cast<int>(kp / factor), (float)static_cast<int>(kd / factor)}, "u32 s16 s16");
    return sendCommand(id, 0x08, trigger_data);
}

bool DrempowerScDriver::motionAid(uint8_t id, float angle, float speed, float angle_err, float speed_err, float torque) {
    float factor = 0.01f;
    if (angle < -300.0f || angle > 300.0f) return false;

    auto data = packData({(float)static_cast<int>(angle / factor), 
                          (float)static_cast<int>(angle_err / factor), 
                          (float)static_cast<int>(speed_err / factor), 
                          (float)static_cast<int>(torque / factor)}, "s16 u16 u16 s16");
    if (!sendCommand(id, 0x0D, data)) return false;

    if (speed <= 0) return false;
    auto speed_preset_data = packData({std::abs(speed), 0.0f, 0.0f}, "f s16 s16");
    if (!sendCommand(id, 0x1A, speed_preset_data)) return false;

    uint32_t order_num = 0x20;
    auto trigger_data = packData({(float)order_num}, "u32");
    return sendCommand(id, 0x08, trigger_data);
}

bool DrempowerScDriver::syncTrigger(uint32_t order_num, uint8_t id) {
    auto trigger_data = packData({(float)order_num, 1.0f}, "u32 u16");
    return sendCommand(id, 0x08, trigger_data);
}

bool DrempowerScDriver::setZeroPosition(uint8_t id) {
    uint32_t order_num = 0x05;
    auto data = packData({(float)order_num}, "u32");
    return sendCommand(id, 0x08, data);
}

bool DrempowerScDriver::setZeroPositionTemp(uint8_t id) {
    uint32_t order_num = 0x23;
    auto data = packData({(float)order_num}, "u32");
    return sendCommand(id, 0x08, data);
}

bool DrempowerScDriver::setAngleRange(uint8_t id, float min_angle, float max_angle) {
    float current_angle = readProperty(id, addr::angle);
    if (std::isnan(current_angle)) return false;
    if (current_angle >= min_angle && current_angle <= max_angle) {
        if (!writeProperty(id, addr::angle_min, min_angle)) return false;
        if (!writeProperty(id, addr::angle_max, max_angle)) return false;
        return writeProperty(id, addr::enable_angle_limit, 1.0f);
    }
    return false;
}

bool DrempowerScDriver::disableAngleRange(uint8_t id) {
    return writeProperty(id, addr::enable_angle_limit, 0.0f);
}

bool DrempowerScDriver::setAngleRangeConfig(uint8_t id, float min_angle, float max_angle) {
    float current_angle = readProperty(id, addr::angle);
    if (std::isnan(current_angle)) return false;
    if (current_angle >= min_angle && current_angle <= max_angle) {
        if (!writeProperty(id, addr::angle_min_config, min_angle)) return false;
        if (!writeProperty(id, addr::angle_max_config, max_angle)) return false;
        return writeProperty(id, addr::enable_angle_limit_config, 1.0f);
    }
    return false;
}

bool DrempowerScDriver::disableAngleRangeConfig(uint8_t id) {
    return writeProperty(id, addr::enable_angle_limit_config, 0.0f);
}

bool DrempowerScDriver::setSpeedLimit(uint8_t id, float speed_limit) {
    if (speed_limit <= 0) return false;
    if (!presetAngle(id, std::abs(speed_limit), 0.0f, 0.0f, 1)) return false;
    uint32_t order_num = 0x18;
    auto data = packData({(float)order_num}, "u32");
    return sendCommand(id, 0x08, data);
}

bool DrempowerScDriver::setTorqueLimit(uint8_t id, float torque_limit) {
    if (torque_limit <= 0) return false;
    if (!presetAngle(id, std::abs(torque_limit), 0.0f, 0.0f, 1)) return false;
    uint32_t order_num = 0x19;
    auto data = packData({(float)order_num}, "u32");
    return sendCommand(id, 0x08, data);
}

bool DrempowerScDriver::setSpeedAdaptiveLimit(uint8_t id, float speed_adaptive) {
    if (speed_adaptive <= 0) return false;
    if (!presetAngle(id, std::abs(speed_adaptive), 0.0f, 0.0f, 1)) return false;
    uint32_t order_num = 0x20;
    auto data = packData({(float)order_num}, "u32");
    return sendCommand(id, 0x08, data);
}

bool DrempowerScDriver::setTorqueAdaptiveLimit(uint8_t id, float torque_adaptive) {
    if (torque_adaptive <= 0) return false;
    if (!presetAngle(id, std::abs(torque_adaptive), 0.0f, 0.0f, 1)) return false;
    uint32_t order_num = 0x21;
    auto data = packData({(float)order_num}, "u32");
    return sendCommand(id, 0x08, data);
}

bool DrempowerScDriver::setPID(uint8_t id, float p, float i, float d) {
    if (p <= 0 || i <= 0 || d <= 0) return false;
    if (!writeProperty(id, addr::angle_gain, p)) return false;
    if (!writeProperty(id, addr::speed_integrator_gain, i)) return false;
    return writeProperty(id, addr::speed_gain, d);
}

bool DrempowerScDriver::setMode(uint8_t id, uint32_t mode) {
    return writeProperty(id, addr::requested_state, static_cast<float>(mode));
}

bool DrempowerScDriver::setCanBaudRate(uint8_t id, uint32_t baud_rate) {
    if (!writeProperty(id, addr::can_baud_rate, static_cast<float>(baud_rate))) return false;
    return saveConfig(id);
}

bool DrempowerScDriver::estop(uint8_t id) {
    uint32_t order_num = 0x06;
    auto data = packData({(float)order_num}, "u32");
    return sendCommand(id, 0x08, data);
}

bool DrempowerScDriver::saveConfig(uint8_t id) {
    uint32_t order_num = 0x01;
    auto data = packData({(float)order_num}, "u32");
    bool ret = sendCommand(id, 0x08, data);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    return ret;
}

bool DrempowerScDriver::disableStateFeedback(uint8_t id) {
    uint16_t address = addr::can_enable_state_feedback;
    uint16_t data_type = static_cast<uint16_t>(DataType::U32);
    uint32_t value = 0;
    auto data = packData({(float)address, (float)data_type, (float)value}, "u16 u16 u32");
    for(int i=0; i<5; ++i) sendCommand(id, 0x1F, data);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    clearBuffer();
    return true;
}

bool DrempowerScDriver::enableStateFeedback(uint8_t id) {
    uint16_t address = addr::can_enable_state_feedback;
    uint16_t data_type = static_cast<uint16_t>(DataType::U32);
    uint32_t value = 1;
    auto data = packData({(float)address, (float)data_type, (float)value}, "u16 u16 u32");
    return sendCommand(id, 0x1F, data);
}

bool DrempowerScDriver::setStateFeedbackRate(uint8_t id, uint16_t interval_ms) {
    uint16_t address = addr::state_feedback_rate_ms;
    uint16_t data_type = static_cast<uint16_t>(DataType::U32);
    auto data = packData({(float)address, (float)data_type, (float)interval_ms}, "u16 u16 u32");
    for(int i=0; i<5; ++i) sendCommand(id, 0x1F, data);
    return true;
}

void DrempowerScDriver::clearBuffer() {
    if (!isOpen()) return;
    
    struct pollfd pfd;
    pfd.fd = can_socket_;
    pfd.events = POLLIN;
    
    struct can_frame frame;
    while (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
        read(can_socket_, &frame, sizeof(struct can_frame));
    }
}

bool DrempowerScDriver::sendCommand(uint8_t id, uint8_t cmd, const std::vector<uint8_t>& data, uint8_t rtr) {
    if (!isOpen()) return false;
    
    struct can_frame frame;
    frame.can_id = (static_cast<uint32_t>(id) << 5) | cmd;
    if (rtr) {
        frame.can_id |= CAN_RTR_FLAG;
    }
    frame.can_dlc = std::min<size_t>(data.size(), 8);
    for (size_t i = 0; i < frame.can_dlc; ++i) {
        frame.data[i] = data[i];
    }
    
    std::lock_guard<std::mutex> lock(can_mutex_);
    
    int tries = 0;
    while (tries < 5) {
        int nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
        if (nbytes == sizeof(struct can_frame)) {
            return true;
        }
        
        if (errno == ENOBUFS || errno == EAGAIN) {
            // Buffer full, wait a bit and retry
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            tries++;
        } else {
            std::cerr << "CAN send error (ID: " << (int)id << ", CMD: 0x" << std::hex << (int)cmd 
                      << "): " << std::strerror(errno) << std::endl;
            return false;
        }
    }
    
    std::cerr << "CAN send failed after retries (ENOBUFS)" << std::endl;
    return false;
}

bool DrempowerScDriver::getMotorState(uint8_t id, MotorStateSC& state) {
    state.id = id;
    state.angle = readProperty(id, addr::angle);
    state.speed = readProperty(id, addr::speed);
    state.torque = readProperty(id, addr::torque);
    
    return !std::isnan(state.angle) && !std::isnan(state.speed);
}

float DrempowerScDriver::readProperty(uint8_t id, uint16_t address) {
    DataType type = get_property_type(address);
    uint16_t type_val = static_cast<uint16_t>(type);
    
    // Clear buffer before requesting a property to ensure we get the fresh response
    clearBuffer();
    
    std::vector<uint8_t> data = packData({(float)address, (float)type_val}, "u16 u16");
    if (!sendCommand(id, 0x1E, data)) return std::numeric_limits<float>::quiet_NaN();
    
    std::vector<uint8_t> resp;
    if (receiveData(resp, id, 0x1E)) {
        std::string format;
        if (type == DataType::F) format = "u16 u16 f";
        else if (type == DataType::U32) format = "u16 u16 u32";
        else if (type == DataType::S16) format = "u16 u16 s16";
        else if (type == DataType::S32) format = "u16 u16 s32";
        
        auto vals = unpackData(resp, format);
        if (!vals.empty()) return vals.back();
    }
    return std::numeric_limits<float>::quiet_NaN();
}

bool DrempowerScDriver::writeProperty(uint8_t id, uint16_t address, float value) {
    DataType type = get_property_type(address);
    uint16_t type_val = static_cast<uint16_t>(type);
    
    std::string format;
    if (type == DataType::F) format = "u16 u16 f";
    else if (type == DataType::U32) format = "u16 u16 u32";
    else if (type == DataType::S16) format = "u16 u16 s16";
    else if (type == DataType::S32) format = "u16 u16 s32";
    
    auto data = packData({(float)address, (float)type_val, value}, format);
    return sendCommand(id, 0x1F, data);
}

bool DrempowerScDriver::receiveData(std::vector<uint8_t>& response, uint8_t expected_id, uint8_t expected_cmd) {
    if (!isOpen()) return false;
    
    struct pollfd pfd;
    pfd.fd = can_socket_;
    pfd.events = POLLIN;
    
    struct can_frame frame;
    size_t tries = 0;
    uint32_t target_can_id = (static_cast<uint32_t>(expected_id) << 5) | expected_cmd;
    
    while (tries < 50) {
        int ret = poll(&pfd, 1, 10); // 10ms timeout
        if (ret > 0 && (pfd.revents & POLLIN)) {
            int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
            if (nbytes == sizeof(struct can_frame)) {
                // IMPORTANT: Verify the CAN ID matches what we requested.
                // This prevents picking up frames from other motors or status reports.
                if (frame.can_id == target_can_id) {
                    response.assign(frame.data, frame.data + frame.can_dlc);
                    while(response.size() < 8) response.push_back(0);
                    return true;
                }
                // If it's not the frame we want, keep looking (don't increment tries)
                continue;
            }
        } else {
            tries++;
        }
    }
    
    return false;
}

std::vector<uint8_t> DrempowerScDriver::packData(const std::vector<float>& values, const std::string& format) {
    std::vector<uint8_t> result;
    std::stringstream ss(format);
    std::string type;
    size_t i = 0;
    while (ss >> type && i < values.size()) {
        if (type == "f") {
            float v = values[i++];
            uint8_t bytes[4];
            std::memcpy(bytes, &v, 4);
            for(int j=0; j<4; ++j) result.push_back(bytes[j]);
        } else if (type == "u16") {
            uint16_t v = static_cast<uint16_t>(values[i++]);
            result.push_back(v & 0xFF);
            result.push_back((v >> 8) & 0xFF);
        } else if (type == "s16") {
            int16_t v = static_cast<int16_t>(values[i++]);
            uint8_t bytes[2];
            std::memcpy(bytes, &v, 2);
            for(int j=0; j<2; ++j) result.push_back(bytes[j]);
        } else if (type == "u32") {
            uint32_t v = static_cast<uint32_t>(values[i++]);
            uint8_t bytes[4];
            std::memcpy(bytes, &v, 4);
            for(int j=0; j<4; ++j) result.push_back(bytes[j]);
        } else if (type == "s32") {
            int32_t v = static_cast<int32_t>(values[i++]);
            uint8_t bytes[4];
            std::memcpy(bytes, &v, 4);
            for(int j=0; j<4; ++j) result.push_back(bytes[j]);
        }
    }
    while (result.size() < 8) result.push_back(0);
    return result;
}

std::vector<float> DrempowerScDriver::unpackData(const std::vector<uint8_t>& data, const std::string& format) {
    std::vector<float> result;
    std::stringstream ss(format);
    std::string type;
    size_t p = 0;
    while (ss >> type && p < data.size()) {
        if (type == "f") {
            if (p + 4 > data.size()) break;
            float v;
            std::memcpy(&v, &data[p], 4);
            result.push_back(v);
            p += 4;
        } else if (type == "u16") {
            if (p + 2 > data.size()) break;
            uint16_t v = data[p] | (data[p+1] << 8);
            result.push_back(static_cast<float>(v));
            p += 2;
        } else if (type == "s16") {
            if (p + 2 > data.size()) break;
            int16_t v;
            std::memcpy(&v, &data[p], 2);
            result.push_back(static_cast<float>(v));
            p += 2;
        } else if (type == "u32") {
            if (p + 4 > data.size()) break;
            uint32_t v;
            std::memcpy(&v, &data[p], 4);
            result.push_back(static_cast<float>(v));
            p += 4;
        } else if (type == "s32") {
            if (p + 4 > data.size()) break;
            int32_t v;
            std::memcpy(&v, &data[p], 4);
            result.push_back(static_cast<float>(v));
            p += 4;
        }
    }
    return result;
}

} // namespace drempower
