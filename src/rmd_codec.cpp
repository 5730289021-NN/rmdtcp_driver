#include <rmd_driver/rmd_codec.h>
#include <ros/ros.h>
#include <algorithm>
#include <chrono>
#include <numeric>

namespace rmd_driver_hardware_interface
{
    std::vector<uint8_t> RMDCodec::encode_command_request(uint8_t motor_id, double cmd) {
        std::vector<uint8_t> request_frame = {
            static_cast<uint8_t>(0x3E), 
            static_cast<uint8_t>(0xA2), 
            motor_id,
            static_cast<uint8_t>(0x04),
            static_cast<uint8_t>(0xE4 + motor_id),
            static_cast<uint8_t>(0x00),
            static_cast<uint8_t>(0x00),
            static_cast<uint8_t>(0x00),
            static_cast<uint8_t>(0x00),
            static_cast<uint8_t>(0x00)
        };
        // Checked
        //vel is in rad/s, require to convert to (degree/minute) then multiply by 10
        //    rad/s                     dpm          
        //vel ----> [ x180 / PI x 60 ] ----> [ x10 ] ---> cmd
        // int32_t speed_control_left = cmd[0] * 180 * 60 * 10 / M_PI;
        // int32_t speed_control_right = cmd[1] * 180 * 60 * 10 / M_PI;
        int32_t speed_command = cmd * 180 * 60 * 10 / M_PI; //raw

        //ROS_INFO("Sending Speed: %f, %d", cmd, speed_command);


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

    double RMDCodec::decode_position_response(uint8_t motor_id, std::vector<uint8_t>& input_buffer) {
        int64_t motor_angle = 0; //raw
        for(size_t i = 0; i < input_buffer.size(); i++) {
            //Assume first byte is the header;
            if(input_buffer[i] != 0) {
                if(input_buffer[i] != 0x3E) {
                    // We don't want to terminate the program when this occur, just warn
                    ROS_WARN("Error occurred on 1st byte!, should be %x, actual %x", 0x3E, input_buffer[i]);
                }
                // for(size_t j = i; j < input_buffer.size(); j++){
                //     //Everything in buffer
                //     ROS_INFO("i = %ld, j = %ld, inbufj = %x", i, j, input_buffer[j]);
                // }
                
                // ROS_ASSERT_MSG(input_buffer[i+1] == 0x92, "Error occurred on 2nd byte!, should be %x, actual %x", 0x92, input_buffer[i+1]);
                // ROS_ASSERT_MSG(input_buffer[i+2] == motor_id, "Error occurred on 3nd byte!, should be %x, actual %x", motor_id, input_buffer[i+2]);
                // ROS_ASSERT_MSG(input_buffer[i+3] == 0x08, "Error occurred on 4nd byte!, should be %x, actual %x", 0x08, input_buffer[i+3]);
                // ROS_ASSERT_MSG(input_buffer[i+4] == CHECKSUM_POS_HEADER_RES_ZO + motor_id, "Error occurred on 4nd byte!, should be %x, actual %x", CHECKSUM_POS_HEADER_RES_ZO + motor_id, input_buffer[i+4]);
                uint8_t checksum = std::accumulate(input_buffer.begin()+i+5, input_buffer.begin()+i+13, 0);
                if(checksum != input_buffer[i+13]) {
                    ROS_ERROR("Wrong Checksum on %d", motor_id);
                }
                // ROS_INFO("Checksum: %x, actual: %x", checksum, input_buffer[i+13]);
                // ROS_ASSERT_MSG(input_buffer[i+13] == checksum, "Error occurred on data checksum byte!, should be %x, actual %x", checksum, input_buffer[i+13]);
                
                // ROS_INFO("----------");
                *(uint8_t *)(&motor_angle) = input_buffer[5];
                *((uint8_t *)(&motor_angle)+1) = input_buffer[6];
                *((uint8_t *)(&motor_angle)+2) = input_buffer[7];
                *((uint8_t *)(&motor_angle)+3) = input_buffer[8];
                *((uint8_t *)(&motor_angle)+4) = input_buffer[9];
                *((uint8_t *)(&motor_angle)+5) = input_buffer[10];
                *((uint8_t *)(&motor_angle)+6) = input_buffer[11];
                *((uint8_t *)(&motor_angle)+7) = input_buffer[12];

                return motor_angle / 216000.0 * M_PI; //Checked
            }
        }
        throw std::runtime_error("Unable to track first byte");
    }

    MotorResponse RMDCodec::decode_command_response(uint8_t motor_id, std::vector<uint8_t>& input_buffer) {
        MotorResponse motor_response;
        for(size_t i = 0; i < input_buffer.size(); i++) {
            if(input_buffer[i] != 0) {
                if(input_buffer[i] != 0x3E) {
                    ROS_WARN("Error occurred on 1st byte!, should be %x, actual %x", 0x3E, input_buffer[i]);
                }
                // for(size_t j = i; j < input_buffer.size(); j++){
                //     //Everything in buffer
                //     ROS_INFO("i = %ld, j = %ld, inbufj = %x", i, j, input_buffer[j]);
                // }
                // ROS_ASSERT_MSG(input_buffer[i+1] == 0xA2, "Error occurred on 2nd byte!, should be %x, actual %x", 0xA2, input_buffer[i+1]);
                // ROS_ASSERT_MSG(input_buffer[i+2] == motor_id, "Error occurred on 3nd byte!, should be %x, actual %x", motor_id, input_buffer[i+2]);
                // ROS_ASSERT_MSG(input_buffer[i+3] == 0x07, "Error occurred on 4nd byte!, should be %x, actual %x", 0x07, input_buffer[i+3]);
                // ROS_ASSERT_MSG(input_buffer[i+4] == CHECKSUM_CMD_HEADER_RES_ZO + motor_id, "Error occurred on 4nd byte!, should be %x, actual %x", CHECKSUM_CMD_HEADER_RES_ZO + motor_id, input_buffer[i+4]);
                uint8_t checksum = std::accumulate(input_buffer.begin()+i+5, input_buffer.begin()+i+12, 0);
                if(checksum != input_buffer[i+12]) {
                    ROS_ERROR("Wrong Checksum on %d", motor_id);
                }
                // ROS_INFO("Checksum: %x, actual: %x", checksum, input_buffer[i+13]);
                // ROS_ASSERT_MSG(input_buffer[i+12] == checksum, "Error occurred on data checksum byte!, should be %x, actual %x", checksum, input_buffer[i+13]);
                // ROS_INFO("----------");
                *(uint8_t *)(&motor_response.temperature) = input_buffer[5];
                *(uint8_t *)(&motor_response.current) = input_buffer[6];
                *((uint8_t *)(&motor_response.current)+1) = input_buffer[7];
                *(uint8_t *)(&motor_response.speed) = input_buffer[8];
                *((uint8_t *)(&motor_response.speed)+1) = input_buffer[9];
                *(uint8_t *)(&motor_response.encoder) = input_buffer[10];
                *((uint8_t *)(&motor_response.encoder)+1) = input_buffer[11];
                return motor_response;
            }
        }
        throw std::runtime_error("Unable to track first byte");
    }

}
