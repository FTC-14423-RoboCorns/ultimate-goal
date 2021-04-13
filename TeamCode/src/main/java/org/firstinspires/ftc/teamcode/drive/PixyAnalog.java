package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PixyAnalog
{
    public double sensorHeight;
    public double fourRing;
    private AnalogInput pixy;
   // private PixyDigital pixydig;
    public PixyAnalog(HardwareMap hardwareMap) {
        init(hardwareMap);
    }

    private void init(HardwareMap hardwareMap) {
        pixy = hardwareMap.get(AnalogInput.class, "PixyAnalog");
  //      pixydig = new PixyDigital(hardwareMap);
        fourRing = pixy.getVoltage();
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
        if(sensorHeight<(fourRing - 1))
        {
            return 0;
        }
        else if(sensorHeight<(fourRing -.05))
        {
            return 1;
        }
        else return 2;

    }

}
