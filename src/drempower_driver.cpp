#include "drempower_sdk/drempower_driver.hpp"
#include "drempower_sdk/parameter_interface.hpp"
#include <cstring>
#include <chrono>
#include <thread>
#include <limits>
#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

namespace drempower {

DrempowerDriver::DrempowerDriver() : serial_port_(nullptr) {}

DrempowerDriver::~DrempowerDriver() {
    close();
}

bool DrempowerDriver::open(const std::string& port, uint32_t baudrate) {
    try {
        serial_port_ = std::make_unique<boost::asio::serial_port>(io_context_, port);
        serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baudrate));
        serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
        serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to open serial port: " << e.what() << std::endl;
        return false;
    }
}

void DrempowerDriver::close() {
    if (serial_port_ && serial_port_->is_open()) {
        serial_port_->close();
    }
}

bool DrempowerDriver::setAngle(uint8_t id, float angle, float speed, float param, uint8_t mode) {
    if (!presetAngle(id, angle, speed, param, mode)) return false;
    uint32_t order_num = 0x08 + mode;
    auto trigger_data = packData({(float)order_num, 1.0f}, "u32 u16");
    return sendCommand(id, 0x08, trigger_data);
}

bool DrempowerDriver::presetAngle(uint8_t id, float angle, float speed, float param, uint8_t mode) {
    float factor = 0.01f;
    uint8_t cmd = 0x19; // Default mode 0
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

bool DrempowerDriver::setSpeed(uint8_t id, float speed, float param, uint8_t mode) {
    if (!presetSpeed(id, speed, param, mode)) return false;
    uint32_t order_num = 0x0B + mode;
    auto trigger_data = packData({(float)order_num, 1.0f}, "u32 u16");
    return sendCommand(id, 0x08, trigger_data);
}

bool DrempowerDriver::presetSpeed(uint8_t id, float speed, float param, uint8_t mode) {
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

bool DrempowerDriver::setTorque(uint8_t id, float torque, float param, uint8_t mode) {
    if (!presetTorque(id, torque, param, mode)) return false;
    uint32_t order_num = 0x0E + mode;
    auto trigger_data = packData({(float)order_num, 1.0f}, "u32 u16");
    return sendCommand(id, 0x08, trigger_data);
}

bool DrempowerDriver::presetTorque(uint8_t id, float torque, float param, uint8_t mode) {
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

bool DrempowerDriver::setAngleAdaptive(uint8_t id, float angle, float speed, float torque) {
    float factor = 0.01f;
    uint8_t cmd = 0x0B;
    int16_t s16_speed = static_cast<int16_t>(std::abs(speed) / factor);
    int16_t s16_torque = static_cast<int16_t>(std::abs(torque) / factor);
    auto data = packData({angle, (float)s16_speed, (float)s16_torque}, "f s16 s16");
    return sendCommand(id, cmd, data);
}

bool DrempowerDriver::stepAngle(uint8_t id, float angle, float speed, float param, uint8_t mode) {
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

bool DrempowerDriver::impedanceControl(uint8_t id, float angle, float speed, float tff, float kp, float kd, uint8_t mode) {
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

    // preset_angle mode 2 => cmd 0x1B
    float f_factor = 0.01f;
    int16_t s16_speed_ff = static_cast<int16_t>(speed / f_factor);
    int16_t s16_torque_ff = static_cast<int16_t>(tff / f_factor);
    auto preset_data = packData({angle_set, (float)s16_speed_ff, (float)s16_torque_ff}, "f s16 s16");
    if (!sendCommand(id, 0x1B, preset_data)) return false;

    uint32_t order_num = 0x15;
    auto trigger_data = packData({(float)order_num, (float)static_cast<int>(kp / factor), (float)static_cast<int>(kd / factor)}, "u32 s16 s16");
    return sendCommand(id, 0x08, trigger_data);
}

bool DrempowerDriver::motionAid(uint8_t id, float angle, float speed, float angle_err, float speed_err, float torque) {
    float factor = 0.01f;
    if (angle < -300.0f || angle > 300.0f) return false;

    auto data = packData({(float)static_cast<int>(angle / factor), 
                          (float)static_cast<int>(angle_err / factor), 
                          (float)static_cast<int>(speed_err / factor), 
                          (float)static_cast<int>(torque / factor)}, "s16 u16 u16 s16");
    if (!sendCommand(id, 0x0D, data)) return false;

    // set_speed_adaptive helper logic
    if (speed <= 0) return false;
    // preset_angle mode 1 => cmd 0x1A
    auto speed_preset_data = packData({std::abs(speed), 0.0f, 0.0f}, "f s16 s16");
    if (!sendCommand(id, 0x1A, speed_preset_data)) return false;

    uint32_t order_num = 0x20;
    auto trigger_data = packData({(float)order_num}, "u32");
    return sendCommand(id, 0x08, trigger_data);
}

bool DrempowerDriver::syncTrigger(uint32_t order_num, uint8_t id) {
    auto trigger_data = packData({(float)order_num, 1.0f}, "u32 u16");
    return sendCommand(id, 0x08, trigger_data);
}

bool DrempowerDriver::setZeroPosition(uint8_t id) {
    uint32_t order_num = 0x05;
    auto data = packData({(float)order_num}, "u32");
    return sendCommand(id, 0x08, data);
}

bool DrempowerDriver::setZeroPositionTemp(uint8_t id) {
    uint32_t order_num = 0x23;
    auto data = packData({(float)order_num}, "u32");
    return sendCommand(id, 0x08, data);
}

bool DrempowerDriver::setAngleRange(uint8_t id, float min_angle, float max_angle) {
    float current_angle = readProperty(id, addr::angle);
    if (std::isnan(current_angle)) return false;
    if (current_angle >= min_angle && current_angle <= max_angle) {
        if (!writeProperty(id, addr::angle_min, min_angle)) return false;
        if (!writeProperty(id, addr::angle_max, max_angle)) return false;
        return writeProperty(id, addr::enable_angle_limit, 1.0f);
    }
    return false;
}

bool DrempowerDriver::disableAngleRange(uint8_t id) {
    return writeProperty(id, addr::enable_angle_limit, 0.0f);
}

bool DrempowerDriver::setAngleRangeConfig(uint8_t id, float min_angle, float max_angle) {
    float current_angle = readProperty(id, addr::angle);
    if (std::isnan(current_angle)) return false;
    if (current_angle >= min_angle && current_angle <= max_angle) {
        if (!writeProperty(id, addr::angle_min_config, min_angle)) return false;
        if (!writeProperty(id, addr::angle_max_config, max_angle)) return false;
        return writeProperty(id, addr::enable_angle_limit_config, 1.0f);
    }
    return false;
}

bool DrempowerDriver::disableAngleRangeConfig(uint8_t id) {
    return writeProperty(id, addr::enable_angle_limit_config, 0.0f);
}

bool DrempowerDriver::setSpeedLimit(uint8_t id, float speed_limit) {
    if (speed_limit <= 0) return false;
    if (!presetAngle(id, std::abs(speed_limit), 0.0f, 0.0f, 1)) return false;
    uint32_t order_num = 0x18;
    auto data = packData({(float)order_num}, "u32");
    return sendCommand(id, 0x08, data);
}

bool DrempowerDriver::setTorqueLimit(uint8_t id, float torque_limit) {
    if (torque_limit <= 0) return false;
    if (!presetAngle(id, std::abs(torque_limit), 0.0f, 0.0f, 1)) return false;
    uint32_t order_num = 0x19;
    auto data = packData({(float)order_num}, "u32");
    return sendCommand(id, 0x08, data);
}

bool DrempowerDriver::setSpeedAdaptiveLimit(uint8_t id, float speed_adaptive) {
    if (speed_adaptive <= 0) return false;
    if (!presetAngle(id, std::abs(speed_adaptive), 0.0f, 0.0f, 1)) return false;
    uint32_t order_num = 0x20;
    auto data = packData({(float)order_num}, "u32");
    return sendCommand(id, 0x08, data);
}

bool DrempowerDriver::setTorqueAdaptiveLimit(uint8_t id, float torque_adaptive) {
    if (torque_adaptive <= 0) return false;
    if (!presetAngle(id, std::abs(torque_adaptive), 0.0f, 0.0f, 1)) return false;
    uint32_t order_num = 0x21;
    auto data = packData({(float)order_num}, "u32");
    return sendCommand(id, 0x08, data);
}

bool DrempowerDriver::setPID(uint8_t id, float p, float i, float d) {
    if (p <= 0 || i <= 0 || d <= 0) return false;
    if (!writeProperty(id, addr::angle_gain, p)) return false;
    if (!writeProperty(id, addr::speed_integrator_gain, i)) return false;
    return writeProperty(id, addr::speed_gain, d);
}

bool DrempowerDriver::setMode(uint8_t id, uint32_t mode) {
    return writeProperty(id, addr::requested_state, static_cast<float>(mode));
}

bool DrempowerDriver::setCanBaudRate(uint8_t id, uint32_t baud_rate) {
    if (!writeProperty(id, addr::can_baud_rate, static_cast<float>(baud_rate))) return false;
    return saveConfig(id);
}

bool DrempowerDriver::estop(uint8_t id) {
    uint32_t order_num = 0x06;
    auto data = packData({(float)order_num}, "u32");
    return sendCommand(id, 0x08, data);
}

bool DrempowerDriver::saveConfig(uint8_t id) {
    uint32_t order_num = 0x01;
    auto data = packData({(float)order_num}, "u32");
    bool ret = sendCommand(id, 0x08, data);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    return ret;
}

bool DrempowerDriver::disableStateFeedback(uint8_t id) {
    uint16_t address = addr::can_enable_state_feedback;
    uint16_t data_type = static_cast<uint16_t>(DataType::U32);
    uint32_t value = 0;
    auto data = packData({(float)address, (float)data_type, (float)value}, "u16 u16 u32");
    for(int i=0; i<5; ++i) sendCommand(id, 0x1F, data);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    clearUart();
    return true;
}

bool DrempowerDriver::enableStateFeedback(uint8_t id) {
    uint16_t address = addr::can_enable_state_feedback;
    uint16_t data_type = static_cast<uint16_t>(DataType::U32);
    uint32_t value = 1;
    auto data = packData({(float)address, (float)data_type, (float)value}, "u16 u16 u32");
    return sendCommand(id, 0x1F, data);
}

bool DrempowerDriver::setStateFeedbackRate(uint8_t id, uint16_t interval_ms) {
    uint16_t address = addr::state_feedback_rate_ms;
    uint16_t data_type = static_cast<uint16_t>(DataType::U32);
    auto data = packData({(float)address, (float)data_type, (float)interval_ms}, "u16 u16 u32");
    for(int i=0; i<5; ++i) sendCommand(id, 0x1F, data);
    return true;
}

void DrempowerDriver::clearUart() {
    if (serial_port_ && serial_port_->is_open()) {
        int fd = serial_port_->native_handle();
        tcflush(fd, TCIFLUSH);
    }
}

bool DrempowerDriver::sendCommand(uint8_t id, uint8_t cmd, const std::vector<uint8_t>& data, uint8_t rtr) {
    if (!isOpen()) return false;
    
    std::vector<uint8_t> udata(16, 0x00);
    udata[0] = 0xAA;
    udata[1] = 0x00;
    udata[2] = rtr ? 0x01 : 0x00;
    udata[3] = 0x08; // DLC
    
    uint16_t id_list = (static_cast<uint16_t>(id) << 5) + cmd;
    udata[6] = (id_list >> 8) & 0xFF;
    udata[7] = id_list & 0xFF;
    
    for (size_t i = 0; i < 8 && i < data.size(); ++i) {
        udata[8 + i] = data[i];
    }
    
    std::lock_guard<std::mutex> lock(serial_mutex_);
    try {
        boost::asio::write(*serial_port_, boost::asio::buffer(udata));
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

bool DrempowerDriver::getMotorState(uint8_t id, MotorState& state) {
    state.id = id;
    state.angle = readProperty(id, addr::angle);
    state.speed = readProperty(id, addr::speed);
    state.torque = readProperty(id, addr::torque);
    
    return !std::isnan(state.angle) && !std::isnan(state.speed);
}

float DrempowerDriver::readProperty(uint8_t id, uint16_t address) {
    DataType type = get_property_type(address);
    uint16_t type_val = static_cast<uint16_t>(type);
    
    std::vector<uint8_t> data = packData({(float)address, (float)type_val}, "u16 u16");
    if (!sendCommand(id, 0x1E, data)) return std::numeric_limits<float>::quiet_NaN();
    
    std::vector<uint8_t> resp;
    if (receiveData(resp)) {
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

bool DrempowerDriver::writeProperty(uint8_t id, uint16_t address, float value) {
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

bool DrempowerDriver::receiveData(std::vector<uint8_t>& response) {
    if (!isOpen()) return false;
    
    std::lock_guard<std::mutex> lock(serial_mutex_);
    uint8_t head = 0;
    size_t tries = 0;
    while (head != 0xAA && tries < 500) {
        boost::asio::read(*serial_port_, boost::asio::buffer(&head, 1));
        tries++;
    }
    if (head != 0xAA) return false;
    
    std::vector<uint8_t> frame(15);
    boost::asio::read(*serial_port_, boost::asio::buffer(frame));
    
    // frame[5] is ID_MSB, frame[6] is ID_LSB, frame[7-14] is DATA0-7
    response.assign(frame.begin() + 7, frame.begin() + 15);
    return true;
}

std::vector<uint8_t> DrempowerDriver::packData(const std::vector<float>& values, const std::string& format) {
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

std::vector<float> DrempowerDriver::unpackData(const std::vector<uint8_t>& data, const std::string& format) {
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
