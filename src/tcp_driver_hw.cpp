#include <rmd_driver/tcp_driver_hw.h>
#include <boost/asio.hpp>
#include <ros/ros.h>
#include <algorithm>
#include <chrono>

namespace rmd_driver_hardware_interface
{
    TCPDriverHW::TCPDriverHW(std::string _ip, std::string _port) : ip_string(_ip), port(_port), socket(io_context){
        // Connect with timeout of 3 secs)
        auto endpoints = boost::asio::ip::tcp::resolver(io_context).resolve(ip_string, port);
        boost::system::error_code error;
        boost::asio::async_connect(socket, endpoints,
            [&](const boost::system::error_code &result_error,
                const boost::asio::ip::tcp::endpoint&)
                {
                    error = result_error;
                });
        run(std::chrono::seconds(3));
        if(error) {
            ROS_ERROR_STREAM("Connection Error: " << error.message());
            throw std::system_error(error);
        }
    }
    
    TCPDriverHW::~TCPDriverHW(){
        socket.close();
    }

    void TCPDriverHW::read(){
        //Wheel Multi-loop angle command Request
        std::vector<uint8_t> req_ang_left_frame = codec.encode_position_request(LEFT_WHEEL_ID);
        std::vector<uint8_t> req_ang_right_frame = codec.encode_position_request(RIGHT_WHEEL_ID);

        if(socket.is_open()) {
            tcp_write(req_ang_left_frame, std::chrono::milliseconds(100)); // Write a request
            tcp_read_amount(14, std::chrono::milliseconds(100)); // data is pushed into input_buffer
            pos[0] = codec.decode_position_response(LEFT_WHEEL_ID, input_buffer);

            tcp_write(req_ang_right_frame, std::chrono::milliseconds(100)); // Write a request
            tcp_read_amount(14, std::chrono::milliseconds(100)); // data is pushed into input_buffer
            pos[1] = codec.decode_position_response(RIGHT_WHEEL_ID, input_buffer);

        } else {
            ROS_ERROR("Socket is not opened yet");
            throw "Socket is not opened yet";
        }
    }

    void TCPDriverHW::write(){
        //vel is in rad/s, require to convert to (degree/minute) then multiply by 10
        //    rad/s                     dpm          
        //vel ----> [ x180 / PI x 60 ] ----> [ x10 ] ---> cmd
        int32_t speed_control_left = cmd[0] * 180 * 60 * 10 / M_PI;
        int32_t speed_control_right = cmd[0] * 180 * 60 * 10 / M_PI;
        
        std::vector<uint8_t> req_spd_cmd_frame_left = codec.encode_command_request(LEFT_WHEEL_ID, cmd[0]);
        std::vector<uint8_t> req_spd_cmd_frame_right = codec.encode_command_request(RIGHT_WHEEL_ID, cmd[1]);

        tcp_write(req_spd_cmd_frame_left, std::chrono::milliseconds(100));
        tcp_write(req_spd_cmd_frame_right, std::chrono::milliseconds(100));
    }

    void TCPDriverHW::tcp_read_amount(size_t n_bytes, std::chrono::steady_clock::duration timeout) {
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

    void TCPDriverHW::tcp_write(std::vector<uint8_t> &message, std::chrono::steady_clock::duration timeout) {
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

    void TCPDriverHW::run(std::chrono::steady_clock::duration timeout) {
        io_context.restart();
        io_context.run_for(timeout);
        // In case of timeout
        if(!io_context.stopped()) {
            socket.close();
            io_context.run();
        }
    }
}