#pragma once 

#include <Eigen/Dense>

using namespace Eigen;

namespace melex_os
{
    class FirstOrderESO
    {                
        double b0_;
        Vector2d L, w;        
        Matrix2d oA;
        Vector2d oB, oC;
        Vector2d xhat;

    public:
        FirstOrderESO(double samplingTime, double b0, double bandwidth, double settlingTime) : b0_(b0)
        {
            Matrix2d Ad;
            Ad<<1, samplingTime, 0, 1;
            Vector2d Bd;
            Bd<<b0_*samplingTime, 0;
            Vector2d Cd;
            Cd<<1, 0;
            double Dd = 0;
            double sCL = -4.0/settlingTime;
            double kp = -2.0*sCL;
            double sESO = bandwidth*sCL;
            double zESO = exp(sESO*samplingTime);
            L<<1 - zESO*zESO, (1.0/samplingTime)*(1 - zESO*zESO)*(1 - zESO*zESO);
            w<<kp/b0,1/b0;
            oA = Ad - L*Cd.transpose()*Ad;
            oB = Bd - L*Cd.transpose()*Bd;
            xhat<<0,0;
        }

        double feedbackUpdate(double currentOutput, double prevControl, double control)
        {
            xhat = oA*xhat + oB*prevControl + L*currentOutput;           
            return control/b0_ - w.transpose()*xhat;
        }
    };

    class SecondOrderESO
    {                
        double b0_;
        Vector3d L, w;        
        Matrix3d oA;
        Vector3d oB, oC;
        Vector3d xhat;

    public:
        SecondOrderESO(double samplingTime, double b0, double bandwidth, double settlingTime) : b0_(b0)
        {
            Matrix3d Ad;
            Ad<<1, samplingTime, samplingTime*samplingTime*0.5,
                 0, 1, samplingTime,
                 0, 0, 1;
            Vector3d Bd;
            Bd<<b0_*samplingTime*samplingTime*0.5, b0_*samplingTime, 0;
            Vector3d Cd;
            Cd<<1, 0, 0;
            double Dd = 0;
            double sCL = -6.0/settlingTime;
            double kp = sCL*sCL;
            double kd = -2.0*sCL;
            double sESO = bandwidth*sCL;
            double zESO = exp(sESO*samplingTime);
            L<<1 - zESO*zESO*zESO, 
                (3.0/(2.0*samplingTime))*(1-zESO)*(1-zESO)*(1+zESO),
                (1.0/(samplingTime*samplingTime))*(1-zESO)*(1-zESO)*(1-zESO);
            w<<kp/b0, kd/b0, 1.0/b0;
            oA = Ad - L*Cd.transpose()*Ad;
            oB = Bd - L*Cd.transpose()*Bd;
            xhat<<0,0;
        }

        double feedbackUpdate(double currentOutput, double prevControl, double control)
        {
            xhat = oA*xhat + oB*prevControl + L*currentOutput;           
            return control/b0_ - w.transpose()*xhat;
        }
    };
}