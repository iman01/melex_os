#pragma once

#include <melex_os/pwm.hpp>
#include <melex_os/steering_encoder.hpp>

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
    

    //coefficient identified using procedure from scripts/calibrate.py
    double columnToBicycleScaling(double columnRad)
    {
        return columnRad*0.07607886;
    }

    public:
        SteeringController(int pigpioId, double loopRate, double steeringAngleLimit) : pwm(pigpioId, 19, 13, 1000), loopRate_(loopRate), steeringAngleLimit_(steeringAngleLimit)
        {

        }
        void update(double referenceSteeringAngle, double referenceSteeringRate, double dutyCycleLimit)
        {
            double steeringAngle = columnToBicycleScaling(encoder.readColumnRad());
            double steeringError = referenceSteeringAngle - steeringAngle;
            double steeringRate = (steeringAngle - prevSteeringAngle)*loopRate_;
            prevSteeringAngle = steeringAngle;
            double steeringRateError = referenceSteeringRate - steeringRate;
            double controlSignal = kd*steeringError + kf*steeringRateError;

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
        }
    };
}