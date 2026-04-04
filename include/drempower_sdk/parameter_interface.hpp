#ifndef DREMPOWER_SDK_PARAMETER_INTERFACE_HPP
#define DREMPOWER_SDK_PARAMETER_INTERFACE_HPP

#include <cstdint>
#include <string>

namespace drempower {

// Data Types mapping as per Python data_types = {'f': 0, 'u16': 1, 's16': 2, 'u32': 3, 's32': 4}
enum class DataType : uint16_t {
    F = 0,
    U16 = 1,
    S16 = 2,
    U32 = 3,
    S32 = 4
};

// Properties Addresses
namespace addr {
    // dr. (Base 00000)
    const uint16_t voltage = 1;
    const uint16_t current = 2;
    
    // can. (Base 20000)
    const uint16_t can_error = 20001;
    
    // can.config (Base 21000)
    const uint16_t can_baud_rate = 21001;
    
    // can.feedback (Base 22000)
    const uint16_t can_enable_state_feedback = 22001;
    
    // dr.state (Base 30000)
    const uint16_t error = 30001;
    const uint16_t current_state = 30002;
    const uint16_t requested_state = 30003;
    const uint16_t version_date = 30004;
    
    // dr.config (Base 31000)
    const uint16_t can_id = 31001;
    const uint16_t state_feedback_rate_ms = 31002;
    const uint16_t product_model = 31003;
    
    // dr.config.angle_limit (Base 31200)
    const uint16_t enable_angle_limit_config = 31201;
    const uint16_t angle_min_config = 31202;
    const uint16_t angle_max_config = 31203;
    const uint16_t gear_ratio = 31204;
    const uint16_t stall_current_limit = 31205;
    const uint16_t enable_crash_detect = 31206;
    const uint16_t crash_detect_sensitivity = 31207;
    const uint16_t enable_encoder_circular_limit = 31208;
    
    // dr.controller (Base 32000)
    const uint16_t controller_error = 32001;
    const uint16_t position_done = 32002;
    const uint16_t position_precision = 32003;
    
    // dr.controller.config (Base 32100)
    const uint16_t enable_speed_limit = 32101;
    const uint16_t angle_gain = 32102;
    const uint16_t speed_gain = 32103;
    const uint16_t speed_integrator_gain = 32104;
    const uint16_t speed_limit = 32105;
    const uint16_t speed_limit_tolerance = 32106;
    const uint16_t inertia = 32107;
    const uint16_t input_filter_bandwidth = 32108;
    
    // dr.motor (Base 33000)
    const uint16_t motor_error = 33001;
    
    // dr.motor.config (Base 33100)
    const uint16_t pole_pairs = 33101;
    const uint16_t phase_inductance = 33102;
    const uint16_t phase_resistance = 33103;
    const uint16_t torque_constant = 33104;
    const uint16_t current_limit = 33105;
    const uint16_t current_limit_margin = 33106;
    const uint16_t torque_limit = 33107;
    const uint16_t current_control_bandwidth = 33108;
    
    // dr.motor.current (Base 33200)
    const uint16_t Iq_measured = 33201;
    const uint16_t Id_measured = 33202;
    
    // dr.encoder (Base 34000)
    const uint16_t encoder_error = 34001;
    const uint16_t abs_output = 34002;
    const uint16_t encoder_pos_zero = 34003;
    const uint16_t abs_angle_power_on = 34004;
    const uint16_t abs_turns_power_on = 34005;
    const uint16_t sign_turns = 34006;
    
    // dr.encoder.output (Base 34100)
    const uint16_t pos_zero_output = 34101;
    const uint16_t abs_pos_power_on = 34102;
    const uint16_t first_half_T = 34103;
    const uint16_t first_half_M = 34104;
    const uint16_t second_half_M = 34105;
    const uint16_t intercept = 34106;
    const uint16_t first_half_up = 34107;
    
    // dr.board_temp (Base 36000)
    const uint16_t board_temp_error = 36001;
    const uint16_t board_temperature = 36002;
    
    // dr.board_temp.config (Base 36100)
    const uint16_t board_temp_enabled = 36101;
    const uint16_t board_temp_limit_lower = 36102;
    const uint16_t board_temp_limit_upper = 36103;
    
    // dr.motor_temp (Base 37000)
    const uint16_t motor_temp_error = 37001;
    const uint16_t motor_temperature = 37002;
    
    // dr.motor_temp.config (Base 37100)
    const uint16_t motor_temp_enabled = 37101;
    const uint16_t motor_temp_limit_lower = 37102;
    const uint16_t motor_temp_limit_upper = 37103;
    
    // dr.output_shaft (Base 38000)
    const uint16_t angle = 38001;
    const uint16_t speed = 38002;
    const uint16_t torque = 38003;
    const uint16_t angle_min = 38004;
    const uint16_t angle_max = 38005;
    const uint16_t enable_angle_limit = 38006;
}

// Property to Type mapping
inline DataType get_property_type(uint16_t address) {
    // Float types
    if (address == addr::voltage || address == addr::current || 
        (address >= 31202 && address <= 31205) || address == addr::crash_detect_sensitivity ||
        address == addr::position_precision || (address >= 32102 && address <= 32108) ||
        (address >= 33102 && address <= 33108) || address == addr::Iq_measured || address == addr::Id_measured ||
        (address >= 34103 && address <= 34106) || address == addr::board_temperature ||
        (address >= 36102 && address <= 36103) || address == addr::motor_temperature ||
        (address >= 37102 && address <= 37103) || (address >= 38001 && address <= 38005)) {
        return DataType::F;
    }
    
    // S32 types
    if (address == addr::pole_pairs || (address >= 34002 && address <= 34003) || 
        address == addr::abs_turns_power_on || (address >= 34101 && address <= 34102)) {
        return DataType::S32;
    }
    
    // U32 types (Default for most configs and errors)
    return DataType::U32;
}

} // namespace drempower

#endif // DREMPOWER_SDK_PARAMETER_INTERFACE_HPP
