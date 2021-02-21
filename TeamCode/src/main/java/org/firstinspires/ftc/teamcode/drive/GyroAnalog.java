package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class GyroAnalog
{
   private double rawAngle;
   private double finalAngle;
    public AnalogInput gyro;
   // private PixyDigital pixydig;
    public GyroAnalog(HardwareMap hardwareMap) {
        init(hardwareMap);
    }

    private void init(HardwareMap hardwareMap) {
        gyro = hardwareMap.get(AnalogInput.class, "Gyro");

    }
    public double readGyro()
    {
        rawAngle=gyro.getVoltage();
        finalAngle=((2*Math.PI*(3.225-(rawAngle-0.003)))/3.225);
        return finalAngle;
    }

}
