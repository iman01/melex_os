#include <ros/ros.h>
#include <stdexcept>
#include <melex_os/CarState.h>
#include <melex_os/CarControl.h>

#include <melex_os/steering_controller.hpp>

#include <pigpiod_if2.h>

int main(int argc, char **argv)
{
  double loopFrequencyHz = 250.0;
  double steeringAngleLimit = M_PI/6;
  using namespace melex_os;

  ros::init(argc, argv, "melex_os");

  ros::NodeHandle nh;

  CarControl controlSignal;
  auto controlSub = nh.subscribe<CarControl>("/melex_os/car_control", 1, [&](const CarControl::ConstPtr& msg){ controlSignal = *msg;});
  
  ros::Publisher statePub = nh.advertise<CarState>("/melex_os/car_state", 1);

  ros::Rate rate(1.0/loopFrequencyHz);


  int pigpioId = -1;
  try
  {
    //initialization
    ROS_INFO("Melex OS driver started, initializing...");
    pigpioId = pigpio_start(NULL, NULL); 
    if(pigpioId < 0)
    {
      throw std::runtime_error("Failed to initialize pigpio. Check if pgpiod is running.");
    }

    SteeringController steeringController(pigpioId, loopFrequencyHz, steeringAngleLimit);    
  
    //control loop
    ROS_INFO("Melex OS driver initialized. Driver is now active.");
  
      while (ros::ok())
      {
        
        steeringController.update(controlSignal.refSteeringAngle, controlSignal.refSteeringRate, 0.5);
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