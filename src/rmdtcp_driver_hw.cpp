#include <rmdtcp_driver/rmdtcp_driver_hw.h>
#include <boost/asio.hpp>
#include <ros/ros.h>

#define LEFT_WHEEL_ID 0x01
#define RIGHT_WHEEL_ID 0x02
#define WAIST_ID 0x03

namespace rmdtcp_driver_hardware_interface
{
    RMDTCP_Driver::RMDTCP_Driver(std::string _ip, std::string _port) : ip_string(_ip), port(_port), socket(io_context){

        // ROS Control Part
        // connect and register the joint state interface
        hardware_interface::JointStateHandle state_handle_waist("waist", &pos[0], &vel[0], &eff[0]);
        jnt_state_interface.registerHandle(state_handle_waist);

        hardware_interface::JointStateHandle state_handle_left_wheel("left_wheel", &pos[1], &vel[1], &eff[1]);
        jnt_state_interface.registerHandle(state_handle_left_wheel);

        hardware_interface::JointStateHandle state_handle_right_wheel("right_wheel", &pos[2], &vel[2], &eff[2]);
        jnt_state_interface.registerHandle(state_handle_right_wheel);

        registerInterface(&jnt_state_interface);

        // connect and register the joint position interface
        // hardware_interface::JointHandle pos_handle_waist(jnt_state_interface.getHandle("waist"), &cmd[0]);
        // jnt_pos_interface.registerHandle(pos_handle_waist);

        // registerInterface(&jnt_pos_interface);

        hardware_interface::JointHandle vel_handle_left_wheel(jnt_state_interface.getHandle("left_wheel"), &cmd[1]);
        jnt_vel_interface.registerHandle(vel_handle_left_wheel);

        hardware_interface::JointHandle vel_handle_right_wheel(jnt_state_interface.getHandle("right_wheel"), &cmd[2]);
        jnt_vel_interface.registerHandle(vel_handle_right_wheel);

        registerInterface(&jnt_vel_interface);

        // Network Part(connect timeout 3 secs)
        tcp_connect(ip_string, port, std::chrono::seconds(3));
    }
    
    RMDTCP_Driver::~RMDTCP_Driver(){
        socket.close();
    }

    void RMDTCP_Driver::read(){
        //Wheel Multi-loop angle command Request
        static std::vector<uint8_t> req_ang_left_frame = { 0x3E, 0x92, LEFT_WHEEL_ID , 0x00, 0x3E + 0x92 + LEFT_WHEEL_ID };
        static std::vector<uint8_t> req_ang_right_frame = { 0x3E, 0x92, RIGHT_WHEEL_ID, 0x00, 0x3E + 0x92 + RIGHT_WHEEL_ID };

        if(socket.is_open()) {
            tcp_write(req_ang_left_frame, std::chrono::milliseconds(100)); // Write a request
            tcp_read_amount(14, std::chrono::milliseconds(100)); // data is pushed into input_buffer
            static uint8_t motor_angle_left = 0;
            //Convert Message for Left Motor
            if( input_buffer[0] == 0x3E &&
                input_buffer[1] == 0x92 &&
                input_buffer[2] == LEFT_WHEEL_ID &&
                input_buffer[3] == 0x08 &&
                input_buffer[4] == 0x3E + 0x92 + LEFT_WHEEL_ID + 0x08)
                {
                    *(uint8_t *)(&motor_angle_left) = input_buffer[5];
                    *((uint8_t *)(&motor_angle_left)+1) = input_buffer[6];
                    *((uint8_t *)(&motor_angle_left)+2) = input_buffer[7];
                    *((uint8_t *)(&motor_angle_left)+3) = input_buffer[8];
                    *((uint8_t *)(&motor_angle_left)+4) = input_buffer[9];
                    *((uint8_t *)(&motor_angle_left)+5) = input_buffer[10];
                    *((uint8_t *)(&motor_angle_left)+6) = input_buffer[11];
                    *((uint8_t *)(&motor_angle_left)+7) = input_buffer[12];
                }
            else {
                ROS_ERROR("Wrong metainfo on left wheel");
                throw "Wrong metainfo on left wheel";
            }
            pos[0] = motor_angle_left / 216000.0 * 2 * M_PI;

            tcp_write(req_ang_right_frame, std::chrono::milliseconds(100)); // Write a request
            tcp_read_amount(14, std::chrono::milliseconds(100)); // data is pushed into input_buffer
            static uint64_t motor_angle_right = 0;
            //Convert Message for Right Motor
            if( input_buffer[0] == 0x3E &&
                input_buffer[1] == 0x92 &&
                input_buffer[2] == RIGHT_WHEEL_ID &&
                input_buffer[3] == 0x08 &&
                input_buffer[4] == 0x3E + 0x92 + RIGHT_WHEEL_ID + 0x08)
                {
                    *(uint8_t *)(&motor_angle_right) = input_buffer[5];
                    *((uint8_t *)(&motor_angle_right)+1) = input_buffer[6];
                    *((uint8_t *)(&motor_angle_right)+2) = input_buffer[7];
                    *((uint8_t *)(&motor_angle_right)+3) = input_buffer[8];
                    *((uint8_t *)(&motor_angle_right)+4) = input_buffer[9];
                    *((uint8_t *)(&motor_angle_right)+5) = input_buffer[10];
                    *((uint8_t *)(&motor_angle_right)+6) = input_buffer[11];
                    *((uint8_t *)(&motor_angle_right)+7) = input_buffer[12];
                }
            else{
                ROS_ERROR("Wrong metainfo on right wheel");
                throw "Wrong metainfo on right wheel";
            }
            pos[1] = motor_angle_right / 216000.0 * 2 * M_PI;

        } else {
            ROS_ERROR("Socket is not opened yet");
            throw "Socket is not opened yet";
        }
    }

    void RMDTCP_Driver::write(){
        //vel is in rad/s, require to convert to (degree/minute) then multiply by 10
        //    rad/s                     dpm          
        //vel ----> [ x180 / PI x 60 ] ----> [ x10 ] ---> cmd
        static int32_t speed_control_left = 0;
        static int32_t speed_control_right = 0;
        
        static std::vector<uint8_t> req_spd_cmd_frame_left = {0x3E, 0xA2, LEFT_WHEEL_ID, 0x04, 0xE5, 0x00, 0x00, 0x00, 0x00, 0x00 };
        static std::vector<uint8_t> req_spd_cmd_frame_right = {0x3E, 0xA2, RIGHT_WHEEL_ID, 0x04, 0xE6, 0x00, 0x00, 0x00, 0x00, 0x00 };

        speed_control_left = cmd[0] * 180 * 60 * 10 / M_PI;
        req_spd_cmd_frame_left[5] = *(uint8_t *)(&speed_control_left);
        req_spd_cmd_frame_left[6] = *((uint8_t *)(&speed_control_left) + 1);
        req_spd_cmd_frame_left[7] = *((uint8_t *)(&speed_control_left) + 2);
        req_spd_cmd_frame_left[8] = *((uint8_t *)(&speed_control_left) + 3);
        req_spd_cmd_frame_left[9] = req_spd_cmd_frame_left[5] + req_spd_cmd_frame_left[6] + req_spd_cmd_frame_left[7] + req_spd_cmd_frame_left[8];

        speed_control_right = cmd[0] * 180 * 60 * 10 / M_PI;
        req_spd_cmd_frame_right[5] = *(uint8_t *) (&speed_control_right);
        req_spd_cmd_frame_right[6] = *((uint8_t *)(&speed_control_right) + 1);
        req_spd_cmd_frame_right[7] = *((uint8_t *)(&speed_control_right) + 2);
        req_spd_cmd_frame_right[8] = *((uint8_t *)(&speed_control_right) + 3);
        req_spd_cmd_frame_right[9] = req_spd_cmd_frame_right[5] + req_spd_cmd_frame_right[6] + req_spd_cmd_frame_right[7] + req_spd_cmd_frame_right[8];

        tcp_write(req_spd_cmd_frame_left, std::chrono::milliseconds(100));
        tcp_write(req_spd_cmd_frame_right, std::chrono::milliseconds(100));
    }

    void RMDTCP_Driver::tcp_read_amount(size_t n_bytes, std::chrono::steady_clock::duration timeout) {
        boost::system::error_code error;
        std::size_t n_transfered;
        boost::asio::async_read(socket, boost::asio::buffer(input_buffer, n_bytes), 
        [&](const boost::system::error_code& res_error, std::size_t bytes_transferred){
            error = res_error;
            n_transfered = bytes_transferred;
        });
        run(timeout);
        if(error) {
            ROS_ERROR_STREAM("Error while reading" << error.message());
            throw std::system_error(error);
        }
        return;
    }

    void RMDTCP_Driver::tcp_write(std::vector<uint8_t> &message, std::chrono::steady_clock::duration timeout) {
        boost::system::error_code error;
        boost::asio::async_write(socket, boost::asio::buffer(message), 
        [&](const boost::system::error_code& res_error, std::size_t){
            error = res_error;
        });
        run(timeout);
        if(error){
            throw std::system_error(error);
        }
    }

    void RMDTCP_Driver::tcp_connect(const std::string& _ip_address, std::string _port, std::chrono::steady_clock::duration timeout){
        auto endpoints = boost::asio::ip::tcp::resolver(io_context).resolve(_ip_address, _port);
        boost::system::error_code error;
        boost::asio::async_connect(socket, endpoints,
            [&](const boost::system::error_code &result_error,
                const boost::asio::ip::tcp::endpoint&)
                {
                    error = result_error;
                });
        run(timeout);
        if(error) {
            ROS_ERROR_STREAM("Connection Error: " << error.message());
            throw std::system_error(error);
        }
    }

    void RMDTCP_Driver::run(std::chrono::steady_clock::duration timeout) {
        io_context.restart();
        io_context.run_for(timeout);
        // In case of timeout
        if(!io_context.stopped()) {
            socket.close();
            io_context.run();
        }
    }
}