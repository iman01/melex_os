#include <ros/ros.h>
#include <stdexcept>
#include <melex_os/CarState.h>
#include <melex_os/CarControl.h>

#include <melex_os/steering_controller.hpp>
#include <melex_os/mode_manager.hpp>
#include <melex_os/speed.hpp>

#include <pigpiod_if2.h>

int main(int argc, char **argv)
{
  double loopFrequencyHz = 250.0;
  double steeringAngleLimit = M_PI/6;
  using namespace melex_os;

  ros::init(argc, argv, "melex_driver");

  ros::NodeHandle nh;

  CarControl controlSignal;
  auto controlSub = nh.subscribe<CarControl>("/melex_os/car_control", 1, [&](const CarControl::ConstPtr& msg){ controlSignal = *msg;});
  CarState carState;
  ros::Publisher statePub = nh.advertise<CarState>("/melex_os/car_state", 1);

  ros::Rate rate(1.0/loopFrequencyHz);

  bool modeState;
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
    ModeManager modeManager(pigpioId, 17, 26, 4);
    Speed speed(pigpioId, 27);
    //control loop
    ROS_INFO("Melex OS driver initialized. Driver is now active.");
  
      while (ros::ok())
      {
        

        ROS_INFO_STREAM("state: "<< controlSignal.autonomyEnabled << " ");
        modeManager.setMode(controlSignal.autonomyEnabled, modeState);
        steeringController.update(controlSignal.refSteeringAngle, controlSignal.refSteeringRate, 0.5);


        carState.autonomyEnabled = modeState;
        carState.velocity = speed.update();
        statePub.publish(carState);
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