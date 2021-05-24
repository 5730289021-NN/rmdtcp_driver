#pragma once
#include <rmd_driver/rmd_driver_hw.h>
#include <boost/asio.hpp>

namespace rmd_driver_hardware_interface{
    class UARTDriverHW : public RMDDriverHW
    {
    public:
        UARTDriverHW(std::string _dev_id, int _baud);
        ~UARTDriverHW();
        void read() override;
        void write() override;

    private:
        void uart_read_amount(size_t n_bytes, std::chrono::steady_clock::duration timeout);
        void uart_write(std::vector<uint8_t> &message, std::chrono::steady_clock::duration timeout);
        void uart_connect(const std::string& host, std::string port, std::chrono::steady_clock::duration timeout);    
        void run(std::chrono::steady_clock::duration timeout);

        // Input 
        std::string dev_id;
        int baud;
        boost::asio::io_context io_context;
        boost::asio::serial_port port;
        uint8_t input_buffer[32]; //Read not beyond 32 byte
    };
}