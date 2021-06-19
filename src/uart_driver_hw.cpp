#include <rmd_driver/uart_driver_hw.h>
#include <boost/asio.hpp>
#include <ros/ros.h>
#include <algorithm>
#include <chrono>

namespace rmd_driver_hardware_interface
{
    UARTDriverHW::UARTDriverHW(std::string _dev_id, int _baud) : dev_id(_dev_id), baud(_baud), port(io_context, _dev_id){
        //Open implicitly from initialization list
        port.set_option(boost::asio::serial_port_base::baud_rate(baud));
        port.set_option(boost::asio::serial_port_base::character_size(8));
        port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        
        ROS_INFO("Successfully open port");
    }
    
    UARTDriverHW::~UARTDriverHW(){
        port.close();
    }

    void UARTDriverHW::read(){
        //Wheel Multi-loop angle command Request
        std::vector<uint8_t> req_ang_left_frame = codec.encode_position_request(LEFT_WHEEL_ID);
        std::vector<uint8_t> req_ang_right_frame = codec.encode_position_request(RIGHT_WHEEL_ID);

        if(port.is_open()) {
            // Send a multiloop position read request
            uart_write(req_ang_left_frame, std::chrono::milliseconds(100)); // Write a position read request
            uart_read_chunk(AMOUNT_POS_FRAME_RES, std::chrono::milliseconds(100));
            pos[0] = codec.decode_position_response(LEFT_WHEEL_ID, input_buffer);
            input_buffer.clear();


            uart_write(req_ang_right_frame, std::chrono::milliseconds(100)); // Write a position read request
            uart_read_chunk(AMOUNT_POS_FRAME_RES, std::chrono::milliseconds(100));
            pos[1] = codec.decode_position_response(RIGHT_WHEEL_ID, input_buffer);
            input_buffer.clear();

        } else {
            ROS_ERROR("Port is not opened yet");
            throw "Port is not opened yet";
        }
    }

    void UARTDriverHW::write(){
        //vel is in rad/s, require to convert to (degree/minute) then multiply by 10
        //    rad/s                     dpm          
        //vel ----> [ x180 / PI x 60 ] ----> [ x10 ] ---> cmd
        // int32_t speed_control_left = cmd[0] * 180 * 60 * 10 / M_PI;
        // int32_t speed_control_right = cmd[1] * 180 * 60 * 10 / M_PI;
        
        std::vector<uint8_t> req_spd_cmd_frame_left = codec.encode_command_request(LEFT_WHEEL_ID, cmd[0]);
        std::vector<uint8_t> req_spd_cmd_frame_right = codec.encode_command_request(RIGHT_WHEEL_ID, cmd[1]);

        if(port.is_open()) {
            uart_write(req_spd_cmd_frame_left, std::chrono::milliseconds(100));
            uart_read_chunk(AMOUNT_CMD_FRAME_RES, std::chrono::milliseconds(100));          
            codec.decode_command_response(LEFT_WHEEL_ID, input_buffer);
            input_buffer.clear();

            uart_write(req_spd_cmd_frame_right, std::chrono::milliseconds(100));
            uart_read_chunk(AMOUNT_CMD_FRAME_RES, std::chrono::milliseconds(100));
            codec.decode_command_response(RIGHT_WHEEL_ID, input_buffer);
            input_buffer.clear();
        } else {
            ROS_ERROR("Port is not opened yet");
            throw "Port is not opened yet";
        }
    }

    // Read n-byte of chunk excluding leading zero
    void UARTDriverHW::uart_read_chunk(size_t n_bytes, std::chrono::steady_clock::duration timeout) {
        boost::system::error_code error;
        std::size_t n_transfered;
        // 3rd parameter could be boost::asio::transfer_at_least
        boost::asio::async_read(port, boost::asio::dynamic_buffer(input_buffer), boost::asio::transfer_at_least(n_bytes),
        // [=] (const boost::system::error_code& error, std::size_t bytes_transferred) -> std::size_t {
        //     for(size_t i = 0; i < bytes_transferred; i++) { // i is number of leading zero
        //         if(input_buffer[i] > 0) { // Detect a character
        //             return (i + n_bytes - bytes_transferred) > 0 ? 
        //             i + n_bytes - bytes_transferred : 0;
        //         }
        //     }
        //     return n_bytes;
        // },
        [&](const boost::system::error_code& res_error, std::size_t bytes_transferred){
             error = res_error;
             n_transfered = bytes_transferred;
        }
        );
        run(timeout);
        if(error) {
            ROS_ERROR_STREAM("Error while reading" << error.message());
            throw std::system_error(error);
        }
    }

    void UARTDriverHW::uart_write(std::vector<uint8_t> &message, std::chrono::steady_clock::duration timeout) {
        boost::system::error_code error;
        boost::asio::async_write(port, boost::asio::buffer(message), 
        [&](const boost::system::error_code& res_error, std::size_t){
            error = res_error;
        });
        run(timeout);
        if(error){
            throw std::system_error(error);
        }
    }

    void UARTDriverHW::run(std::chrono::steady_clock::duration timeout) {
        io_context.restart();
        io_context.run_for(timeout);
        // In case of timeout
        if(!io_context.stopped()) {
            ROS_ERROR("Operation Timeout, Probably no data back drom device.");
            ROS_ERROR("Current Buffer (Size = %ld) is :", input_buffer.size());
            for(size_t i = 0; i < input_buffer.size(); i++) {
                ROS_ERROR("%x", input_buffer[i]);
            }
            port.close();
            io_context.run();
        }
    }
}