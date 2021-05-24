#pragma once
#include <cstdint>
#include <vector>

namespace rmd_driver_hardware_interface
{
    struct MotorResponse{
        int8_t temperature;
        int16_t current;
        int16_t speed;
        uint16_t encoder;
    };

    class RMDCodec {
        public:
            std::vector<uint8_t> encode_command_request(uint8_t motor_id, double cmd);
            std::vector<uint8_t> encode_position_request(uint8_t motor_id);
            double decode_position_response(uint8_t motor_id, uint8_t* input_buffer);
            MotorResponse decode_command_response(uint8_t motor_id, uint8_t* input_buffer);
    };
}