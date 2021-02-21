package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class PixyAnalog
{
    public double sensorHeight;
    public double oneRing;
    private AnalogInput pixy;
   // private PixyDigital pixydig;
    public PixyAnalog(HardwareMap hardwareMap) {
        init(hardwareMap);
    }

    private void init(HardwareMap hardwareMap) {
        pixy = hardwareMap.get(AnalogInput.class, "PixyAnalog");
  //      pixydig = new PixyDigital(hardwareMap);
        oneRing = pixy.getVoltage();
    }
    private double readPixy()
    {
        return pixy.getVoltage();
    }
    public int getStackHeight()
    {
/*
        if (!pixydig.ringSensed())
        {return 0;}
        else {
            sensorHeight = readPixy();
            if (sensorHeight < (oneRing - 0.01)) {
                return 1;
            } else {
                return 2;
            }
        }
*/

        sensorHeight = readPixy();
        if(sensorHeight<(oneRing - 1))
        {
            return 0;
        }
        else if(sensorHeight<(oneRing -.05))
        {
            return 1;
        }
        else return 2;

    }

}
