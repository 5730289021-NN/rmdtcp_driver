#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

namespace rmdtcp_driver_hardware_interface{
    class RMDTCP_Driver : public hardware_interface::RobotHW
    {
    public:
        RMDTCP_Driver();
        ~RMDTCP_Driver();
        void read();
        void write();
    private:
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::PositionJointInterface jnt_pos_interface;
        hardware_interface::VelocityJointInterface jnt_vel_interface;
        double cmd[3];
        double pos[3];
        double vel[3];
        double eff[3];
        async_comm::TCPClient* tcp_client;
    };
}