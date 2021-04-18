#pragma once

#include <melex_os/pwm.hpp>
#include <melex_os/steering_encoder.hpp>
#include <melex_os/digital_filters.hpp>
//#include <melex_os/adrc_eso.hpp>

namespace melex_os
{
    class SteeringController
    {

    double loopRate_; 
    double steeringAngleLimit_;   
    //BCM pin numbering mode
    SteeringEncoder encoder;
    PWM  pwm;
    double kd = 1.0;
    double kf = 0.5;

    double prevSteeringAngle = 0.0;
    LowPassFilter3 steeringRateFilter;
    
    double prevControl = 0.0;
    //SecondOrderESO eso;
    

    //coefficient identified using procedure from scripts/calibrate.py
    double columnToBicycleScaling(double columnRad)
    {
        return columnRad*0.07607886;
    }

    public:
        SteeringController(int pigpioId, double loopRate, double steeringAngleLimit) 
        : pwm(pigpioId, 19, 13, 1000), loopRate_(loopRate), steeringAngleLimit_(steeringAngleLimit),
        steeringRateFilter(1.0/loopRate, 200.0)
        {

        }
        //eso(1.0/loopRate, 1.0, 100.0, 0.1)
      
        void update(double referenceSteeringAngle, double referenceSteeringRate, double dutyCycleLimit)
        {
            double steeringAngle = columnToBicycleScaling(encoder.readColumnRad());
            double steeringError = referenceSteeringAngle - steeringAngle;
            double steeringRate = steeringRateFilter.update((steeringAngle - prevSteeringAngle)*loopRate_);
            prevSteeringAngle = steeringAngle;
            double steeringRateError = referenceSteeringRate - steeringRate;
            
            //TODO: try dual output maybe (uncomment those 2 lines and modify ESO)
            // Eigen::Vector2d output;
            // output<<steeringAngle, steeringError;
            double controlSignal = kd*steeringError + kf*steeringRateError;//eso.feedbackUpdate(steeringAngle, prevControl, kd*steeringError + kf*steeringRateError);

            if(controlSignal > dutyCycleLimit)
            {
                controlSignal = dutyCycleLimit;
            } 
            else if(controlSignal < -dutyCycleLimit)
            {
                controlSignal = -dutyCycleLimit;
            } 

            if(fabs(steeringAngle) >= steeringAngleLimit_)
            {
                controlSignal = 0.0;
                ROS_WARN("Steering angle limit reached");
            }


            pwm.setSignedDutyCycle(controlSignal);
            prevControl = controlSignal;
        }
    };
}