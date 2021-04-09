#include <rmdtcp_driver/rmdtcp_driver_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rmdtcp_driver");
  rmdtcp_driver_hardware_interface::RMDTCP_Driver driver("192.168.1.23", "9001");
  controller_manager::ControllerManager cm(&driver);
  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time prev_time = ros::Time::now();
  ros::Rate rate(10.0);

  ROS_INFO("RMDTCP Server Started");

  while (true)
  {
    const ros::Time time = ros::Time::now();
    const ros::Duration period = time - prev_time;
    ROS_INFO_ONCE("Reading Motor Position");
    driver.read();
    cm.update(time, period);
    ROS_INFO_ONCE("Writing Motor Velocity");
    driver.write();
    rate.sleep();
  }

  return 0;
}