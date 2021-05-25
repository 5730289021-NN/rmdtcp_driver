#include <rmd_driver/rmd_codec.h>
#include <ros/ros.h>
#include <algorithm>
#include <chrono>

namespace rmd_driver_hardware_interface
{
    std::vector<uint8_t> RMDCodec::encode_command_request(uint8_t motor_id, double cmd) {
        std::vector<uint8_t> request_frame = {
            static_cast<uint8_t>(0x3E), 
            static_cast<uint8_t>(0xA2), 
            motor_id,
            static_cast<uint8_t>(0x04),
            static_cast<uint8_t>(0xE5),
            static_cast<uint8_t>(0x00),
            static_cast<uint8_t>(0x00),
            static_cast<uint8_t>(0x00),
            static_cast<uint8_t>(0x00),
            static_cast<uint8_t>(0x00)
        };

        int32_t speed_command = cmd * 180 * 60 * 10 / M_PI; //raw
        request_frame[5] = *(uint8_t *)(&speed_command);
        request_frame[6] = *((uint8_t *)(&speed_command) + 1);
        request_frame[7] = *((uint8_t *)(&speed_command) + 2);
        request_frame[8] = *((uint8_t *)(&speed_command) + 3);
        request_frame[9] = request_frame[5] + request_frame[6] + request_frame[7] + request_frame[8];
        
        return request_frame;
    }

    std::vector<uint8_t> RMDCodec::encode_position_request(uint8_t motor_id) {
        std::vector<uint8_t> request_frame = {
            static_cast<uint8_t>(0x3E), 
            static_cast<uint8_t>(0x92),
            motor_id,
            static_cast<uint8_t>(0x00), 
            static_cast<uint8_t>(0xD0 + motor_id)
        };

        return request_frame;
    }

    double RMDCodec::decode_position_response(uint8_t motor_id, uint8_t* input_buffer) {
        int64_t motor_angle = 0; //raw
        if( input_buffer[0] == 0x3E &&
            input_buffer[1] == 0x92 &&
            input_buffer[2] == motor_id &&
            input_buffer[3] == 0x08 &&
            input_buffer[4] == 0xD8 + motor_id)
        {
            *(uint8_t *)(&motor_angle) = input_buffer[5];
            *((uint8_t *)(&motor_angle)+1) = input_buffer[6];
            *((uint8_t *)(&motor_angle)+2) = input_buffer[7];
            *((uint8_t *)(&motor_angle)+3) = input_buffer[8];
            *((uint8_t *)(&motor_angle)+4) = input_buffer[9];
            *((uint8_t *)(&motor_angle)+5) = input_buffer[10];
            *((uint8_t *)(&motor_angle)+6) = input_buffer[11];
            *((uint8_t *)(&motor_angle)+7) = input_buffer[12];
        }
        else {
            ROS_ERROR("Wrong data coming from motor");
            throw "Wrong metainfo on left wheel";
        }
        return motor_angle / 216000.0 * 2 * M_PI;
    }

    MotorResponse RMDCodec::decode_command_response(uint8_t motor_id, uint8_t* input_buffer) {
        MotorResponse motor_response;
        if( input_buffer[0] == 0x3E &&
            input_buffer[1] == 0xA2 &&
            input_buffer[2] == motor_id &&
            input_buffer[3] == 0x07 &&
            input_buffer[4] == 0xE7 + motor_id) 
        {
            *(uint8_t *)(&motor_response.temperature) = input_buffer[5];
            *(uint8_t *)(&motor_response.current) = input_buffer[6];
            *((uint8_t *)(&motor_response.current)+1) = input_buffer[7];
            *(uint8_t *)(&motor_response.speed) = input_buffer[8];
            *((uint8_t *)(&motor_response.speed)+1) = input_buffer[9];
            *(uint8_t *)(&motor_response.encoder) = input_buffer[10];
            *((uint8_t *)(&motor_response.encoder)+1) = input_buffer[11];
        }
        
        return motor_response;
    }

}