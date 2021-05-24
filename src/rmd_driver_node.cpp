#include <rmd_driver/tcp_driver_hw.h>
#include <rmd_driver/uart_driver_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

#define USE_SERIAL
//#define USE_TCP

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rmdtcp_driver");
  
  //rmd_driver_hardware_interface::RMDTCP_Driver driver("192.168.16.21", "9001");

  rmd_driver_hardware_interface::RMDDriverHW* driver;

  #if defined(USE_SERIAL)
  driver = new rmd_driver_hardware_interface::TCPDriverHW("192.168.16.21", "9001");
  #elif defined(USE_TCP)
  driver = new rmd_driver_hardware_interface::UARTDriverHW("/dev/ttyUSB0", 115200);
  #endif

  controller_manager::ControllerManager cm(driver);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time prev_time = ros::Time::now();
  ros::Rate rate(10.0);

  while (true)
  {
    const ros::Time time = ros::Time::now();
    const ros::Duration period = time - prev_time;
    driver->read();
    cm.update(time, period);
    driver->write();
    rate.sleep();
  }

  return 0;
}