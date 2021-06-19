#include <rmd_driver/tcp_driver_hw.h>
#include <rmd_driver/uart_driver_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <string>

#define USE_UART
//#define USE_TCP

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rmdtcp_driver");
  
  //rmd_driver_hardware_interface::RMDTCP_Driver driver("192.168.16.21", "9001");

  rmd_driver_hardware_interface::RMDDriverHW* driver;

  #if defined(USE_UART)
  std::string dev = "/dev/ttyUSB0";//"/dev/serial/by-path/pci-0000\\:00\\:14.0-usb-0\\:2\\:1.0-port0";
  int baud = 115200;
  ROS_INFO("Initializing \"UART\" Mode from device %s with baudrate %d", dev.c_str(), baud);
  driver = new rmd_driver_hardware_interface::UARTDriverHW(dev, baud);
  #elif defined(USE_TCP)
  std::string ip = "192.168.16.21"
  int port = 9001;
  ROS_INFO("Initializing \"TCP\" Mode from IP %s with port %d", ip.c_str(), port);
  driver = new rmd_driver_hardware_interface::TCPDriverHW(ip, port);
  #endif
  ROS_INFO("Initialization successfully");

  controller_manager::ControllerManager cm(driver);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time prev_time = ros::Time::now();
  ros::Rate rate(20.0);

  while (ros::ok())
  {
    const ros::Time time = ros::Time::now();
    const ros::Duration period = time - prev_time;
    //ROS_INFO("Read");
    driver->read();
    cm.update(time, period);
    //ROS_INFO("Write");
    driver->write();
    rate.sleep();
  }

  return 0;
}