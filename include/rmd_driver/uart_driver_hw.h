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
        // void uart_read_amount(size_t n_bytes, std::chrono::steady_clock::duration timeout);
        // void uart_read_until(char delim, std::chrono::steady_clock::duration timeout);
        void uart_read_chunk(size_t n_bytes, std::chrono::steady_clock::duration timeout);
        void uart_write(std::vector<uint8_t> &message, std::chrono::steady_clock::duration timeout);
        void run(std::chrono::steady_clock::duration timeout) override;

        // Input 
        std::string dev_id;
        int baud;

        boost::asio::serial_port port;
    };
}