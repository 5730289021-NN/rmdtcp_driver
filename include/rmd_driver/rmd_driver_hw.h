#pragma once
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <rmd_driver/rmd_codec.h>

#define LEFT_WHEEL_ID 0x01
#define RIGHT_WHEEL_ID 0x02
#define WAIST_ID 0x03

/*
RMD Motor Configurations
ID1: Left Wheel     -> [0]
ID2: Right Wheel    -> [1]
ID3: Waist          -> [2]
*/

namespace rmd_driver_hardware_interface{
    class RMDDriverHW : public hardware_interface::RobotHW 
    {
        public:
            virtual void read();
            virtual void write();

        protected:
            RMDCodec codec;

            hardware_interface::JointStateInterface jnt_state_interface;
            //hardware_interface::PositionJointInterface jnt_pos_interface;
            hardware_interface::VelocityJointInterface jnt_vel_interface;

            double cmd[3];
            double pos[3];
            double vel[3];
            double eff[3];
    };
}