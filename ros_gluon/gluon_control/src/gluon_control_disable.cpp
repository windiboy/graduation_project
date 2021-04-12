#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <gluon_control/gluon_hw_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gluon_disable");
  ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Create the hardware interface specific to your robot
  boost::shared_ptr<gluon_control::GluonHWInterface> gluon_hw_interface
    (new gluon_control::GluonHWInterface(nh));
  gluon_hw_interface->init();

  gluon_hw_interface->disable_all();
  // Start the control loop
  // ros_control_boilerplate::GenericHWControlLoop control_loop(nh, gluon_hw_interface);
  // control_loop.run(); // Blocks until shutdown signal recieved

  return 0;
}
