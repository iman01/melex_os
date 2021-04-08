#include "ros/ros.h"

#include <melex_os/CarState.h>
#include <melex_os/CarControl.h>

#include <modbus/modbus-rtu.h>
#include <pigpiod_if2.h>

int main(int argc, char **argv)
{
  double loopFrequencyHz = 250.0;
  //BCM pin numbering mode
  int dirPin = 19;
  int pwmPin = 13;
  int pwmFreq = 1000;

  using namespace melex_os;

  ros::init(argc, argv, "melex_os");

  ros::NodeHandle nh;

  CarControl controlSignal;
  auto controlSub = nh.subscribe<CarControl>("/melex_os/car_control", 1, [&](const CarControl::ConstPtr& msg){ controlSignal = *msg;});
  
  ros::Publisher statePub = nh.advertise<CarState>("/melex_os/car_state", 1);

  ros::Rate rate(1.0/loopFrequencyHz);

  //initialization
  ROS_INFO("Melex OS driver started, initializing...");
  int pigpioId = pigpio_start(NULL, NULL);
  if(pigpioId < 0)
  {
    ROS_ERROR("Failed to initialize pigpio. Check if pgpiod is running.");
  }

  if(set_mode(pigpioId, dirPin, PI_OUTPUT) < 0)
  {
    ROS_ERROR("Failed to set up steering direction pin");
  }

  if(set_mode(pigpioId, pwmPin, PI_OUTPUT) < 0)
  {
    ROS_ERROR("Failed to set up  steering pwm pin");
  }

  modbus_t* ctx = modbus_new_rtu("/dev/ttyS0", 19200, 'E', 8, 1);
  if(ctx == NULL)
  {
    ROS_ERROR("Failed to initialize steering encoder modbus interface");
  }
  
  //control loop
  ROS_INFO("Melex OS driver initialized. Driver is now active.");
  try
  {
    while (ros::ok())
    {
      rate.sleep();
    }
  }
  catch(const std::exception& e)
  {
    ROS_ERROR_STREAM("Melex OS driver error: " << e.what());
  }

  pigpio_stop(pigpioId);

  ROS_INFO("Melex OS driver exited cleanly.");
  return 0;
}