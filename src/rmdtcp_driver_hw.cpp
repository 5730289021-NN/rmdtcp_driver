#include <rmdtcp_driver/rmdtcp_driver_hw.h>
#include <async_comm/tcp_client.h>

#define LEFT_WHEEL_ID 0x01
#define RIGHT_WHEEL_ID 0x02

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
        tcp_client.close();
        delete tcp_client;
    }

    struct MotorResponse {
        uint8_t id;
        uint8_t response_mode;
        int64_t motor_angle_raw;
    } ;

    void callback(const uint8_t* buf, size_t len){
        static uint8_t response_index = 0;
        static struct MotorResponse mr;
        for(int i = 0; i < len; i++) {
            switch(response_index){
                case 0: { // Header
                    if(buf[i] == 0x3E){
                        continue;
                    } else {
                        ROS_ERROR("Incorrect Header");
                    }
                    break;
                }
                case 1: { // Command
                    switch(buf[i]){
                        case 0xA2: {
                            break;
                        }
                        case 0x92: {
                            response_mode = 0x92;
                            break;
                        }
                        default: {
                            ROS_ERROR("Response not supported");
                        }
                    }
                    break;
                }
                case 2: { // ID
                    mr.id = buf[i];
                }
                case 3: { // Data Length 
                    if(buf[i] != 8){
                        ROS_ERROR("Length not supported");
                    }
                    break;
                }
                case 4: { // Frame Header Check
                    if(buf[i] != 0x3E + 0x92 + mr.id + 0x08){
                        ROS_ERROR("Header Checksum Error");
                    }
                    break;
                }
                
                case 5:
                case 6:
                case 7:
                case 8:
                case 9:
                case 10:
                case 11:
                case 12: { // Angle B1(LSB) to B8
                    // mr.motor_angle_raw
                    // Check Degree?
                    // 
                    if(mr.response_mode == 0x92){
                        *((uint8_t *) (&mr.motor_angle_raw) + response_index - 5) = buf[i];
                    }
                    break;
                }
                case 13: { // B1 - B8 Checksum
                    uint8_t chksum = 0x00;
                    for(int j = 0; j < 8; j++){
                        chksum += *((uint8_t *) (&mr.motor_angle_raw)+j);
                    }
                    if(chksum == buf[i]){
                        //Record
                        pos[mr.id] = 
                    } else {
                        ROS_ERROR("Checksum Mismatched");
                    }
                    break;
                }
                default: {
                    ROS_ERROR("Unknown State");
                }
        }
        }
        
        if(!reading_flag) {
            switch()
            if(buf[0] != 0x3E){
                ROS_ERROR("Incorrect Header Message");
                return;
            } else {
                
            }
        } else{

        }
    }

    void RMDTCP_Driver::read(){
        //Wheel Multi-loop angle command Request
        static uint8_t req_ang_frame[2][5] = {
            { 0x3E, 0x92, LEFT_WHEEL_ID , 0x00, 0x3E + 0x92 + LEFT_WHEEL_ID },
            { 0x3E, 0x92, RIGHT_WHEEL_ID, 0x00, 0x3E + 0x92 + RIGHT_WHEEL_ID }
        };
        for(int i = 0; i < 2; i++){
            tcp_client.send_bytes(req_ang_frame[i], 5);
        }
    }

    void RMDTCP_Driver::write(){
        //vel is in rad/s, require to convert to (degree/minute) then multiply by 10
        //    rad/s                     dpm          
        //vel ----> [ x180 / PI x 60 ] ----> [ x10 ] ---> cmd
        static int32_t speedControl[2];
        static uint8_t req_spd_cmd_frame[2][10] = {
            { 0x3E, 0xA2, LEFT_WHEEL_ID, 0x04, 0xE5, 0x00, 0x00, 0x00, 0x00, 0x00 },
            { 0x3E, 0xA2, RIGHT_WHEEL_ID, 0x04, 0xE6, 0x00, 0x00, 0x00, 0x00, 0x00 }
        };

        for(int i = 0; i < 2; i++){
            speedControl[i] = cmd[i] * 180 * 60 * 10 / M_PI;
            req_spd_cmd_frame[i][5] = *(uint8_t *)(&speedControl[i]);
            req_spd_cmd_frame[i][6] = *((uint8_t *)(&speedControl[i]) + 1);
            req_spd_cmd_frame[i][7] = *((uint8_t *)(&speedControl[i]) + 2);
            req_spd_cmd_frame[i][8] = *((uint8_t *)(&speedControl[i]) + 3);
            req_spd_cmd_frame[i][9] = req_spd_cmd_frame[i][5] + req_spd_cmd_frame[i][6] + req_spd_cmd_frame[i][7] + req_spd_cmd_frame[i][8];
            tcp_client.send_bytes(req_spd_cmd_frame[i], 10);
        }
    }
}