#include <rmdtcp_driver/rmdtcp_driver_hw.h>
#include <async_comm/tcp_client.h>

namespace rmdtcp_driver_hardware_interface
{
    RMDTCP_Driver::RMDTCP_Driver(){
        // connect and register the joint state interface
        hardware_interface::JointStateHandle state_handle_waist("waist", &pos[0], &vel[0], &eff[0]);
        jnt_state_interface.registerHandle(state_handle_waist);

        hardware_interface::JointStateHandle state_handle_left_wheel("left_wheel", &pos[1], &vel[1], &eff[1]);
        jnt_state_interface.registerHandle(state_handle_left_wheel);

        hardware_interface::JointStateHandle state_handle_right_wheel("right_wheel", &pos[2], &vel[2], &eff[2]);
        jnt_state_interface.registerHandle(state_handle_right_wheel);

        registerInterface(&jnt_state_interface);

        // connect and register the joint position interface
        hardware_interface::JointHandle pos_handle_waist(jnt_state_interface.getHandle("waist"), &cmd[0]);
        jnt_pos_interface.registerHandle(pos_handle_waist);

        registerInterface(&jnt_pos_interface);

        hardware_interface::JointHandle vel_handle_left_wheel(jnt_state_interface.getHandle("left_wheel"), &cmd[1]);
        jnt_vel_interface.registerHandle(vel_handle_left_wheel);

        hardware_interface::JointHandle vel_handle_right_wheel(jnt_state_interface.getHandle("right_wheel"), &cmd[2]);
        jnt_vel_interface.registerHandle(vel_handle_right_wheel);

        registerInterface(&jnt_vel_interface);


        tcp_client = new async_comm::TCPClient("localhost", 16140);
        tcp_client.register_receive_callback(std::bind(&RMDTCP_Driver::tcp_callback, this, std::placeholders::_1, std::placeholders::_2));
        if (!tcp_client.init())
        {
            std::cout << "Failed to initialize TCP client" << std::endl;
            return 1;
        }


    }
    
    RMDTCP_Driver::~RMDTCP_Driver(){
        delete tcp_client;
    }

    void callback(const uint8_t* buf, size_t len){
        
    }

    void RMDTCP_Driver::read(){

    }

    void RMDTCP_Driver::write(){

    }
}