#include "ros/ros.h"

#include <melex_os/CarState.h>
#include <melex_os/CarControl.h>

#include <modbus/modbus-rtu.h>
#include <pigpiod_if2.h>

int main(int argc, char **argv)
{
  double loopFrequencyHz = 250.0;
  //BCM pin numbering mode
  unsigned int dirPin = 19;
  unsigned int pwmPin = 13;
  unsigned int pwmFrequency = 1000;

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

  if(set_PWM_frequency(pigpioId, pwmPin, pwmFrequency) < 0)
  {
    ROS_ERROR("Failed to set up  steering pwm  frequency");
  }

  modbus_t* ctx = modbus_new_rtu("/dev/ttyS0", 19200, 'E', 8, 1);
  if(ctx == NULL)
  {
    ROS_ERROR("Failed to initialize steering encoder modbus interface");
  }

  if(modbus_set_slave(ctx, 127) == -1)
  {
    ROS_ERROR("Failed to initialize steering encoder modbus slave");
  } 

  if(modbus_connect(ctx) == -1)
  {
    ROS_ERROR("Failed to connect steering ecnoder modbus slave");
  }

  //TODO: move to a function when we verify it works
  uint16_t registers[2];
  if(modbus_read_registers(ctx, 1, 2, registers) == -1)
  {
    ROS_ERROR("Failed to read encoder value.");
  }
  else
  {
    int turns = registers[0];
    int position = registers[1];
    
    if(turns > 16384)
    {
      turns -= 32768;
    }

    double degrees = position/65536.0*360.0;
    if(turns < 0)
    {
      degrees = 360 - degrees + 360*turns;
    }
    else
    {
      degrees + 360*turns;
    }
    double columnAngleRad = 0.0174532925*degrees;

    ROS_INFO_STREAM("Calibrating steering zero to: " << degrees);
  }

  //control loop
  ROS_INFO("Melex OS driver initialized. Driver is now active.");
  try
  {
    while (ros::ok())
    {
      
      //set_PWM_dutycycle 0-255
      //set direction pin
      rate.sleep();
    }
  }
  catch(const std::exception& e)
  {
    ROS_ERROR_STREAM("Melex OS driver error: " << e.what());
  }

  pigpio_stop(pigpioId);
  modbus_close(ctx);
  modbus_free(ctx);

  ROS_INFO("Melex OS driver exited cleanly.");
  return 0;
}