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
    public PixyAnalog(HardwareMap hardwareMap) {
        init(hardwareMap);
    }

    private void init(HardwareMap hardwareMap) {
        //TODO: Add lift and confirm mag class
        pixy = hardwareMap.get(AnalogInput.class, "PixyAnalog");
        oneRing = pixy.getVoltage();
    }
    private double readPixy()
    {
        return pixy.getVoltage();
    }
    public int getStackHeight()
    {
        sensorHeight = readPixy();
        if(sensorHeight<(oneRing - 0.2))
        {
            return 0;
        }
        else if(sensorHeight<(oneRing + 0.04))
        {
            return 1;
        }
        else return 2;
    }

}
