#include <rmd_driver/rmd_driver_hw.h>
#include <ros/ros.h>
#include <algorithm>
#include <chrono>

namespace rmd_driver_hardware_interface
{
    RMDDriverHW::RMDDriverHW() {

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
    }
    
    RMDDriverHW::~RMDDriverHW(){
    }
}