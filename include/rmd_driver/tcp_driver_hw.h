#pragma once
#include <string>
#include <rmd_driver/rmd_driver_hw.h>
#include <boost/asio.hpp>

namespace rmd_driver_hardware_interface{
    class TCPDriverHW : public RMDDriverHW
    {
    public:
        TCPDriverHW(std::string _ip, std::string _port);
        ~TCPDriverHW();
        void read() override;
        void write() override;

    private:
        void tcp_read_amount(size_t n_bytes, std::chrono::steady_clock::duration timeout);
        void tcp_write(std::vector<uint8_t> &message, std::chrono::steady_clock::duration timeout);

        void run(std::chrono::steady_clock::duration timeout);
        // Boost Asio TCP
        // Input 
        std::string ip_string;
        std::string port;

        boost::asio::io_context io_context;
        boost::asio::ip::tcp::socket socket{io_context};
        uint8_t input_buffer[32]; //Read not beyond 32 byte
    };
}