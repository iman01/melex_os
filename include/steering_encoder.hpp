#pragma once

#include <stdexcept>
#include <ros/ros.h>
#include <modbus/modbus-rtu.h>

namespace melex_os
{
    class SteeringEncoder
    {
        modbus_t* ctx = NULL;
        double columnZero = 0.0;
        double columnRad = 0.0;

    public:
        SteeringEncoder()
        {
            ctx = modbus_new_rtu("/dev/ttyS0", 19200, 'E', 8, 1);
            if(ctx == NULL)
            {
                throw std::runtime_error("Failed to initialize steering encoder modbus interface");
            }

            if(modbus_set_slave(ctx, 127) == -1)
            {
                throw std::runtime_error("Failed to initialize steering encoder modbus slave");
            } 

            if(modbus_connect(ctx) == -1)
            {
                throw std::runtime_error("Failed to connect steering ecnoder modbus slave");
            }
            
            columnZero = readColumnRad();
            columnRad = 0.0;

        }

        ~SteeringEncoder()
        {
            if(ctx != NULL)
            {
                modbus_close(ctx);
                modbus_free(ctx);
            }            
        }


        double readColumnRad()
        {                        
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
                    degrees = 360 - degrees + 360*(turns);
                }
                else
                {
                    degrees = degrees + 360*(turns);
                }
                
                columnRad = 0.0174532925*degrees;                
            }
            return columnRad;
        }
    };
}