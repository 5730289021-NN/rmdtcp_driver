#pragma once
#include <cstdint>
#include <vector>

#define CHECKSUM_POS_HEADER_RES_ZO 216
#define CHECKSUM_CMD_HEADER_RES_ZO 231

#define AMOUNT_POS_FRAME_RES 14
#define AMOUNT_CMD_FRAME_RES 8 

#define AMOUNT_POS_DATAFRAME_READ 9 //Including checksum 
#define AMOUNT_CMD_DATAFRAME_WRITE 8 //Including checksum 

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
            double decode_position_response(uint8_t motor_id, std::vector<uint8_t>& input_buffer);
            MotorResponse decode_command_response(uint8_t motor_id, std::vector<uint8_t>& input_buffer);
    };
}