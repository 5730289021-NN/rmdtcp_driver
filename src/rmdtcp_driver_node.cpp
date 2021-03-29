#include <rmdtcp_driver/rmdtcp_driver_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

int main()
{
  rmdtcp_driver_hardware_interface::RMDTCP_Driver driver("192.168.16.21", "9001");
  controller_manager::ControllerManager cm(&driver);

  

  while (true)
  {
     driver.read(
     cm.update(ros::Time::now(), );
     driver.write();
  }
}