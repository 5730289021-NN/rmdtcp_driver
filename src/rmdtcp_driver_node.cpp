#include <rmdtcp_driver/rmdtcp_driver_hw.h>
#include <controller_manager/controller_manager.h>

int main()
{
  rmdtcp_driver_hardware_interface::RMDTCP_Driver driver;
  controller_manager::ControllerManager cm(&driver);

  while (true)
  {
    const ros::Duration period(1.0);
     driver.read();
     cm.update(driver.get_time(), driver.get_period());
     driver.write();
  }
}