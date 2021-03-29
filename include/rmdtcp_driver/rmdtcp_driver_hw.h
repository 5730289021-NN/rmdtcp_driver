#pragma once
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <boost/asio.hpp>

namespace rmdtcp_driver_hardware_interface{
    class RMDTCP_Driver : public hardware_interface::RobotHW
    {
    public:
        RMDTCP_Driver(std::string _ip, std::string _port);
        ~RMDTCP_Driver();
        void read(const ros::Time& _time, const ros::Duration& _period) override;
        void write(const ros::Time& _time, const ros::Duration& _period) override;

        ros::Time& get_time() const;
        ros::Duration& get_period() const;

        void tcp_read_amount(size_t n_bytes, std::chrono::steady_clock::duration timeout);
        void tcp_write(std::vector<uint8_t> &message, std::chrono::steady_clock::duration timeout);
        void tcp_connect(const std::string& host, std::string port, std::chrono::steady_clock::duration timeout);


    private:
        void run(std::chrono::steady_clock::duration timeout);
        hardware_interface::JointStateInterface jnt_state_interface;
        //hardware_interface::PositionJointInterface jnt_pos_interface;
        hardware_interface::VelocityJointInterface jnt_vel_interface;
        /*
        RMD Motor
        ID1: Left Wheel     -> [0]
        ID2: Right Wheel    -> [1]
        ID3: Waist          -> [2]
        */
        double cmd[3];
        double pos[3];
        double vel[3];
        double eff[3];

        ros::Duration read_period{0};
        ros::Duration write_period{0};
        
        ros::Time time;
        ros::Duration period;
        
        // Boost Asio TCP
        // Input 
        std::string ip_string;
        std::string port;

        boost::asio::io_context io_context;
        boost::asio::ip::tcp::socket socket{io_context};
        uint8_t input_buffer[32]; //Read not beyond 32 byte

    };
}